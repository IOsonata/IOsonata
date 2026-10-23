/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 */
#include <string.h>

#include "crypto/x509.h"

#define X509_MAX_EXTENSIONS		32U
#define X509_KEY_DIGITAL_SIGNATURE	0x8000U
#define X509_KEY_AGREEMENT			0x0800U
#define X509_KEY_CERT_SIGN			0x0400U
#define X509_KEY_ENCIPHER_ONLY		0x0100U
#define X509_KEY_DECIPHER_ONLY		0x0080U

typedef struct {
	const uint8_t *pData;
	uint32_t Len;
} X509Span_t;

typedef struct {
	X509Span_t Raw;
	X509Span_t Value;
	uint8_t Tag;
} X509Der_t;

typedef struct {
	X509Span_t Tbs;
	X509Span_t Issuer;
	X509Span_t Subject;
	X509Span_t Signature;
	const uint8_t *pPublicKey;
	int64_t NotBefore;
	int64_t NotAfter;
	uint32_t PathLen;
	uint16_t Usage;
	bool bBasic;
	bool bBasicCritical;
	bool bCa;
	bool bPathLen;
	bool bUsage;
	bool bEku;
	bool bServerAuth;
	bool bSan;
	bool bSanCritical;
	bool bHostMatch;
	bool bSubjectEmpty;
} X509Info_t;

static const uint8_t s_EcdsaSha256[] = {
	0x2a, 0x86, 0x48, 0xce, 0x3d, 0x04, 0x03, 0x02
};
static const uint8_t s_EcPublicKey[] = {
	0x2a, 0x86, 0x48, 0xce, 0x3d, 0x02, 0x01
};
static const uint8_t s_Prime256v1[] = {
	0x2a, 0x86, 0x48, 0xce, 0x3d, 0x03, 0x01, 0x07
};
static const uint8_t s_ServerAuth[] = {
	0x2b, 0x06, 0x01, 0x05, 0x05, 0x07, 0x03, 0x01
};
static const uint8_t s_AnyEku[] = {
	0x55, 0x1d, 0x25, 0x00
};

/* One bounded DER item. High-tag-number forms are outside this profile. */
static bool X509DerNext(X509Span_t *pInput, X509Der_t *pItem)
{
	uint32_t pos = 2U;
	uint32_t len;
	uint32_t count;
	uint32_t i;

	if (pInput->Len < 2U || pInput->pData[0] == 0U ||
		(pInput->pData[0] & 0x1fU) == 0x1fU)
	{
		return false;
	}
	len = pInput->pData[1];
	if ((len & 0x80U) != 0U)
	{
		count = len & 0x7fU;
		if (count == 0U || count > 4U || count > pInput->Len - pos ||
			pInput->pData[pos] == 0U)
		{
			return false;
		}
		len = 0U;
		for (i = 0U; i < count; i++)
		{
			len = (len << 8) | pInput->pData[pos++];
		}
		if (len < 128U)
		{
			return false;
		}
	}
	if (len > pInput->Len - pos)
	{
		return false;
	}
	pItem->Tag = pInput->pData[0];
	pItem->Raw.pData = pInput->pData;
	pItem->Raw.Len = pos + len;
	pItem->Value.pData = pInput->pData + pos;
	pItem->Value.Len = len;
	pInput->pData += pos + len;
	pInput->Len -= pos + len;
	return true;
}

static bool X509DerRead(X509Span_t *pInput, uint8_t Tag, X509Der_t *pItem)
{
	return X509DerNext(pInput, pItem) && pItem->Tag == Tag;
}

static bool X509Equal(X509Span_t A, X509Span_t B)
{
	return A.Len == B.Len && memcmp(A.pData, B.pData, A.Len) == 0;
}

static bool X509OidEqual(X509Span_t Oid, const uint8_t *pValue, uint32_t Len)
{
	return Oid.Len == Len && memcmp(Oid.pData, pValue, Len) == 0;
}

static bool X509OidValid(X509Span_t Oid)
{
	uint32_t i;
	bool first = true;

	if (Oid.Len == 0U)
	{
		return false;
	}
	for (i = 0U; i < Oid.Len; i++)
	{
		if (first && Oid.pData[i] == 0x80U)
		{
			return false;
		}
		first = (Oid.pData[i] & 0x80U) == 0U;
	}
	return first;
}

static bool X509PositiveInt(X509Span_t Value)
{
	if (Value.Len == 0U || (Value.pData[0] & 0x80U) != 0U)
	{
		return false;
	}
	return Value.Len == 1U || Value.pData[0] != 0U ||
		(Value.pData[1] & 0x80U) != 0U;
}

static bool X509Uint(X509Span_t Value, uint32_t *pNumber)
{
	uint32_t i;
	uint32_t number = 0U;

	if (!X509PositiveInt(Value))
	{
		return false;
	}
	if (Value.Len > 1U && Value.pData[0] == 0U)
	{
		Value.pData++;
		Value.Len--;
	}
	if (Value.Len > 4U)
	{
		return false;
	}
	for (i = 0U; i < Value.Len; i++)
	{
		number = (number << 8) | Value.pData[i];
	}
	*pNumber = number;
	return true;
}

static bool X509EcdsaInt(X509Span_t *pInput, uint8_t *pScalar)
{
	X509Der_t integer;
	X509Span_t value;

	if (!X509DerRead(pInput, 0x02U, &integer) ||
		!X509PositiveInt(integer.Value))
	{
		return false;
	}
	value = integer.Value;
	if (value.Len > 1U && value.pData[0] == 0U)
	{
		value.pData++;
		value.Len--;
	}
	if (value.Len > 32U)
	{
		return false;
	}
	memcpy(pScalar + 32U - value.Len, value.pData, value.Len);
	return P256ScalarInRange(pScalar);
}

bool X509EcdsaP256Decode(const uint8_t *pDer, uint32_t Len,
						 uint8_t pSignature[64])
{
	X509Span_t input = { pDer, Len };
	X509Der_t sequence;
	bool valid;

	if (pSignature == nullptr)
	{
		return false;
	}
	memset(pSignature, 0, 64U);
	if (pDer == nullptr || Len < 8U || Len > 72U ||
		!X509DerRead(&input, 0x30U, &sequence) || input.Len != 0U)
	{
		return false;
	}
	input = sequence.Value;
	valid = X509EcdsaInt(&input, pSignature) &&
		X509EcdsaInt(&input, pSignature + 32U) && input.Len == 0U;
	if (!valid)
	{
		memset(pSignature, 0, 64U);
	}
	return valid;
}

static X509_STATUS X509SignatureAlgorithm(X509Span_t *pInput)
{
	X509Der_t sequence;
	X509Der_t oid;

	if (!X509DerRead(pInput, 0x30U, &sequence) ||
		!X509DerRead(&sequence.Value, 0x06U, &oid) ||
		!X509OidValid(oid.Value))
	{
		return X509_STATUS_FORMAT;
	}
	if (!X509OidEqual(oid.Value, s_EcdsaSha256, sizeof(s_EcdsaSha256)))
	{
		return X509_STATUS_UNSUPPORTED;
	}
	/* RFC 5758: ECDSA AlgorithmIdentifier parameters are absent. */
	return sequence.Value.Len == 0U ? X509_STATUS_OK : X509_STATUS_FORMAT;
}

static bool X509Ascii(X509Span_t Value)
{
	uint32_t i;

	for (i = 0U; i < Value.Len; i++)
	{
		if (Value.pData[i] == 0U || Value.pData[i] > 0x7fU)
		{
			return false;
		}
	}
	return Value.Len != 0U;
}

/* Only characters and canonical encodings used by directory strings. Names
 * are compared as complete DER values; no locale or Unicode normalization. */
static bool X509DirectoryString(const X509Der_t *pString)
{
	X509Span_t value = pString->Value;
	uint32_t i = 0U;
	uint32_t code;
	uint32_t count;
	uint32_t min;
	uint8_t ch;

	if (value.Len == 0U)
	{
		return false;
	}
	if (pString->Tag == 0x0cU)
	{
		while (i < value.Len)
		{
			ch = value.pData[i++];
			if (ch < 0x80U)
			{
				if (ch == 0U)
				{
					return false;
				}
				continue;
			}
			if (ch >= 0xc2U && ch <= 0xdfU)
			{
				code = ch & 0x1fU;
				count = 1U;
				min = 0x80U;
			}
			else if (ch >= 0xe0U && ch <= 0xefU)
			{
				code = ch & 0x0fU;
				count = 2U;
				min = 0x800U;
			}
			else if (ch >= 0xf0U && ch <= 0xf4U)
			{
				code = ch & 7U;
				count = 3U;
				min = 0x10000U;
			}
			else
			{
				return false;
			}
			if (count > value.Len - i)
			{
				return false;
			}
			while (count-- > 0U)
			{
				ch = value.pData[i++];
				if ((ch & 0xc0U) != 0x80U)
				{
					return false;
				}
				code = (code << 6) | (ch & 0x3fU);
			}
			if (code < min || code > 0x10ffffU ||
				(code >= 0xd800U && code <= 0xdfffU))
			{
				return false;
			}
		}
		return true;
	}
	if (pString->Tag == 0x16U)
	{
		return X509Ascii(value);
	}
	if (pString->Tag == 0x13U || pString->Tag == 0x14U)
	{
		for (i = 0U; i < value.Len; i++)
		{
			ch = value.pData[i];
			if (ch == 0U)
			{
				return false;
			}
			if (pString->Tag == 0x13U &&
				!((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
				(ch >= '0' && ch <= '9') ||
				strchr(" '()+,-./:=?", ch) != nullptr))
			{
				return false;
			}
		}
		return true;
	}
	if (pString->Tag == 0x1eU || pString->Tag == 0x1cU)
	{
		count = pString->Tag == 0x1eU ? 2U : 4U;
		if (value.Len % count != 0U)
		{
			return false;
		}
		while (i < value.Len)
		{
			code = (uint32_t)value.pData[i] << 8 | value.pData[i + 1U];
			i += 2U;
			if (count == 4U)
			{
				code = (code << 16) | (uint32_t)value.pData[i] << 8 |
					value.pData[i + 1U];
				i += 2U;
			}
			if (code == 0U || code > 0x10ffffU ||
				(code >= 0xd800U && code <= 0xdfffU))
			{
				return false;
			}
		}
		return true;
	}
	return false;
}

static bool X509Name(X509Span_t *pInput, X509Span_t *pName, bool *pEmpty)
{
	X509Der_t sequence;
	X509Der_t rdn;
	X509Der_t attribute;
	X509Der_t oid;
	X509Der_t value;
	X509Span_t previous;
	uint32_t len;
	int order;

	if (!X509DerRead(pInput, 0x30U, &sequence))
	{
		return false;
	}
	*pName = sequence.Raw;
	*pEmpty = sequence.Value.Len == 0U;
	while (sequence.Value.Len != 0U)
	{
		if (!X509DerRead(&sequence.Value, 0x31U, &rdn) ||
			rdn.Value.Len == 0U)
		{
			return false;
		}
		previous.Len = 0U;
		while (rdn.Value.Len != 0U)
		{
			if (!X509DerRead(&rdn.Value, 0x30U, &attribute))
			{
				return false;
			}
			if (previous.Len != 0U)
			{
				len = previous.Len < attribute.Raw.Len ?
					previous.Len : attribute.Raw.Len;
				order = memcmp(previous.pData, attribute.Raw.pData, len);
				if (order > 0 || (order == 0 && previous.Len > attribute.Raw.Len))
				{
					return false;
				}
			}
			previous = attribute.Raw;
			if (!X509DerRead(&attribute.Value, 0x06U, &oid) ||
				!X509OidValid(oid.Value) ||
				!X509DerNext(&attribute.Value, &value) ||
				attribute.Value.Len != 0U || !X509DirectoryString(&value))
			{
				return false;
			}
		}
	}
	return true;
}

static uint8_t X509Lower(uint8_t Character)
{
	return Character >= 'A' && Character <= 'Z' ?
		(uint8_t)(Character + ('a' - 'A')) : Character;
}

static bool X509DnsValid(X509Span_t Name, bool bWildcard)
{
	uint32_t i;
	uint32_t label = 0U;
	uint8_t ch;
	bool numeric = true;
	bool wildcard = false;

	if (Name.Len == 0U || Name.Len > X509_MAX_HOSTNAME_SIZE)
	{
		return false;
	}
	for (i = 0U; i < Name.Len; i++)
	{
		ch = Name.pData[i];
		if (ch == '.')
		{
			if (label == 0U || Name.pData[i - 1U] == '-')
			{
				return false;
			}
			label = 0U;
			continue;
		}
		if (bWildcard && i == 0U && ch == '*' &&
			Name.Len > 2U && Name.pData[1] == '.')
		{
			wildcard = true;
			label = 1U;
			numeric = false;
			continue;
		}
		if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
			(ch >= '0' && ch <= '9') || ch == '-') ||
			(label == 0U && ch == '-') || ++label > 63U)
		{
			return false;
		}
		if (ch < '0' || ch > '9')
		{
			numeric = false;
		}
	}
	if (label == 0U || Name.pData[Name.Len - 1U] == '-' || numeric)
	{
		return false;
	}
	/* Do not accept a wildcard at a one-label suffix, such as *.com. */
	if (wildcard && memchr(Name.pData + 2U, '.', Name.Len - 2U) == nullptr)
	{
		return false;
	}
	return true;
}

static bool X509DnsMatch(X509Span_t Pattern, X509Span_t Host)
{
	uint32_t i;
	uint32_t offset = 0U;

	if (Pattern.Len > 2U && Pattern.pData[0] == '*')
	{
		while (offset < Host.Len && Host.pData[offset] != '.')
		{
			offset++;
		}
		if (offset == 0U || offset == Host.Len)
		{
			return false;
		}
		/* Compare the suffix including its leading dot. */
		Pattern.pData++;
		Pattern.Len--;
	}
	if (Pattern.Len != Host.Len - offset)
	{
		return false;
	}
	for (i = 0U; i < Pattern.Len; i++)
	{
		if (X509Lower(Pattern.pData[i]) != X509Lower(Host.pData[offset + i]))
		{
			return false;
		}
	}
	return true;
}

static bool X509Time(X509Span_t *pInput, int64_t *pTime)
{
	static const uint16_t monthDays[] = {
		0U, 31U, 59U, 90U, 120U, 151U, 181U, 212U, 243U, 273U, 304U, 334U
	};
	static const uint8_t daysInMonth[] = {
		31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U
	};
	X509Der_t item;
	uint32_t field[7] = {};
	uint32_t count;
	uint32_t i;
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
	uint32_t maximum;
	bool leap;
	int64_t days;

	if (!X509DerNext(pInput, &item) ||
		!((item.Tag == 0x17U && item.Value.Len == 13U) ||
		(item.Tag == 0x18U && item.Value.Len == 15U)) ||
		item.Value.pData[item.Value.Len - 1U] != 'Z')
	{
		return false;
	}
	count = (item.Value.Len - 1U) / 2U;
	for (i = 0U; i < count; i++)
	{
		if (item.Value.pData[2U * i] < '0' ||
			item.Value.pData[2U * i] > '9' ||
			item.Value.pData[2U * i + 1U] < '0' ||
			item.Value.pData[2U * i + 1U] > '9')
		{
			return false;
		}
		field[i] = 10U * (item.Value.pData[2U * i] - '0') +
			item.Value.pData[2U * i + 1U] - '0';
	}
	if (count == 7U)
	{
		year = field[0] * 100U + field[1];
		if (year < 2050U)
		{
			return false;
		}
		i = 2U;
	}
	else
	{
		year = field[0] + (field[0] < 50U ? 2000U : 1900U);
		i = 1U;
	}
	month = field[i++];
	day = field[i++];
	hour = field[i++];
	minute = field[i++];
	second = field[i];
	if (month < 1U || month > 12U || day == 0U || hour > 23U ||
		minute > 59U || second > 59U)
	{
		return false;
	}
	leap = year % 4U == 0U && (year % 100U != 0U || year % 400U == 0U);
	maximum = daysInMonth[month - 1U] + ((leap && month == 2U) ? 1U : 0U);
	if (day > maximum)
	{
		return false;
	}
	days = ((int64_t)year - 1970) * 365 +
		(int64_t)((year - 1U) / 4U - (year - 1U) / 100U +
		(year - 1U) / 400U) - 477;
	days += monthDays[month - 1U] + day - 1U;
	if (leap && month > 2U)
	{
		days++;
	}
	*pTime = days * 86400 + hour * 3600U + minute * 60U + second;
	return true;
}

static X509_STATUS X509PublicKey(X509Span_t *pInput, const uint8_t **ppKey)
{
	X509Der_t sequence;
	X509Der_t algorithm;
	X509Der_t oid;
	X509Der_t curve;
	X509Der_t key;

	if (!X509DerRead(pInput, 0x30U, &sequence) ||
		!X509DerRead(&sequence.Value, 0x30U, &algorithm) ||
		!X509DerRead(&algorithm.Value, 0x06U, &oid) ||
		!X509OidValid(oid.Value))
	{
		return X509_STATUS_FORMAT;
	}
	if (!X509OidEqual(oid.Value, s_EcPublicKey, sizeof(s_EcPublicKey)))
	{
		return X509_STATUS_UNSUPPORTED;
	}
	if (!X509DerRead(&algorithm.Value, 0x06U, &curve) ||
		!X509OidValid(curve.Value) || algorithm.Value.Len != 0U)
	{
		return X509_STATUS_FORMAT;
	}
	if (!X509OidEqual(curve.Value, s_Prime256v1, sizeof(s_Prime256v1)))
	{
		return X509_STATUS_UNSUPPORTED;
	}
	if (!X509DerRead(&sequence.Value, 0x03U, &key) ||
		sequence.Value.Len != 0U || key.Value.Len != 66U ||
		key.Value.pData[0] != 0U || key.Value.pData[1] != 0x04U)
	{
		return X509_STATUS_FORMAT;
	}
	/* SignEngine validates curve membership when the key verifies a signature. */
	*ppKey = key.Value.pData + 2U;
	return X509_STATUS_OK;
}

static X509_STATUS X509BasicConstraints(X509Span_t Input, bool bCritical,
									  X509Info_t *pInfo)
{
	X509Der_t sequence;
	X509Der_t item;

	if (!X509DerRead(&Input, 0x30U, &sequence) || Input.Len != 0U)
	{
		return X509_STATUS_FORMAT;
	}
	pInfo->bBasic = true;
	pInfo->bBasicCritical = bCritical;
	if (sequence.Value.Len != 0U && sequence.Value.pData[0] == 0x01U)
	{
		/* DER omits the default false value. */
		if (!X509DerRead(&sequence.Value, 0x01U, &item) ||
			item.Value.Len != 1U || item.Value.pData[0] != 0xffU)
		{
			return X509_STATUS_FORMAT;
		}
		pInfo->bCa = true;
	}
	if (sequence.Value.Len != 0U)
	{
		if (!X509DerRead(&sequence.Value, 0x02U, &item) ||
			!X509Uint(item.Value, &pInfo->PathLen) || sequence.Value.Len != 0U)
		{
			return X509_STATUS_FORMAT;
		}
		if (!pInfo->bCa)
		{
			return X509_STATUS_CONSTRAINT;
		}
		pInfo->bPathLen = true;
	}
	return X509_STATUS_OK;
}

static X509_STATUS X509KeyUsage(X509Span_t Input, X509Info_t *pInfo)
{
	X509Der_t bits;
	uint8_t last;
	uint8_t unused = 0U;

	if (!X509DerRead(&Input, 0x03U, &bits) || Input.Len != 0U ||
		bits.Value.Len < 2U || bits.Value.Len > 3U ||
		bits.Value.pData[0] > 7U)
	{
		return X509_STATUS_FORMAT;
	}
	last = bits.Value.pData[bits.Value.Len - 1U];
	if (last == 0U || (bits.Value.Len == 3U && last != 0x80U))
	{
		return X509_STATUS_FORMAT;
	}
	while ((last & 1U) == 0U)
	{
		unused++;
		last >>= 1;
	}
	if (bits.Value.pData[0] != unused)
	{
		return X509_STATUS_FORMAT;
	}
	pInfo->bUsage = true;
	pInfo->Usage = (uint16_t)bits.Value.pData[1] << 8;
	if (bits.Value.Len == 3U)
	{
		pInfo->Usage |= bits.Value.pData[2];
	}
	if ((pInfo->Usage & (X509_KEY_ENCIPHER_ONLY | X509_KEY_DECIPHER_ONLY)) != 0U &&
		(pInfo->Usage & X509_KEY_AGREEMENT) == 0U)
	{
		return X509_STATUS_CONSTRAINT;
	}
	return X509_STATUS_OK;
}

static X509_STATUS X509ExtendedKeyUsage(X509Span_t Input, X509Info_t *pInfo)
{
	X509Der_t sequence;
	X509Der_t oid;
	X509Span_t seen[X509_MAX_EXTENSIONS];
	uint32_t count = 0U;
	uint32_t i;

	if (!X509DerRead(&Input, 0x30U, &sequence) || Input.Len != 0U ||
		sequence.Value.Len == 0U)
	{
		return X509_STATUS_FORMAT;
	}
	pInfo->bEku = true;
	while (sequence.Value.Len != 0U)
	{
		if (count == X509_MAX_EXTENSIONS)
		{
			return X509_STATUS_LIMIT;
		}
		if (!X509DerRead(&sequence.Value, 0x06U, &oid) ||
			!X509OidValid(oid.Value))
		{
			return X509_STATUS_FORMAT;
		}
		for (i = 0U; i < count; i++)
		{
			if (X509Equal(oid.Value, seen[i]))
			{
				return X509_STATUS_FORMAT;
			}
		}
		seen[count++] = oid.Value;
		if (X509OidEqual(oid.Value, s_ServerAuth, sizeof(s_ServerAuth)) ||
			X509OidEqual(oid.Value, s_AnyEku, sizeof(s_AnyEku)))
		{
			pInfo->bServerAuth = true;
		}
	}
	return X509_STATUS_OK;
}

static X509_STATUS X509GeneralNames(X509Span_t Input, X509Span_t Host,
								   bool *pMatch)
{
	X509Der_t item;

	if (Input.Len == 0U)
	{
		return X509_STATUS_FORMAT;
	}
	while (Input.Len != 0U)
	{
		if (!X509DerNext(&Input, &item))
		{
			return X509_STATUS_FORMAT;
		}
		switch (item.Tag)
		{
			case 0x82U:
				if (!X509DnsValid(item.Value, true))
				{
					return X509_STATUS_FORMAT;
				}
				if (Host.Len != 0U && X509DnsMatch(item.Value, Host))
				{
					*pMatch = true;
				}
				break;
			case 0x81U:
			case 0x86U:
				if (!X509Ascii(item.Value))
				{
					return X509_STATUS_FORMAT;
				}
				break;
			case 0x87U:
				if (item.Value.Len != 4U && item.Value.Len != 16U)
				{
					return X509_STATUS_FORMAT;
				}
				break;
			case 0x88U:
				if (!X509OidValid(item.Value))
				{
					return X509_STATUS_FORMAT;
				}
				break;
			default:
				return X509_STATUS_UNSUPPORTED;
		}
	}
	return X509_STATUS_OK;
}

static X509_STATUS X509SubjectAltName(X509Span_t Input, X509Span_t Host,
									 bool bCritical, X509Info_t *pInfo)
{
	X509Der_t sequence;

	if (!X509DerRead(&Input, 0x30U, &sequence) || Input.Len != 0U)
	{
		return X509_STATUS_FORMAT;
	}
	pInfo->bSan = true;
	pInfo->bSanCritical = bCritical;
	return X509GeneralNames(sequence.Value, Host, &pInfo->bHostMatch);
}

static X509_STATUS X509AuthorityKeyId(X509Span_t Input)
{
	X509Der_t sequence;
	X509Der_t item;
	X509Der_t generalName;
	X509Span_t name;
	X509Span_t noHost = { nullptr, 0U };
	uint8_t previous = 0U;
	uint8_t index;
	bool issuer = false;
	bool serial = false;
	bool match = false;
	bool empty;
	X509_STATUS status;

	if (!X509DerRead(&Input, 0x30U, &sequence) || Input.Len != 0U ||
		sequence.Value.Len == 0U)
	{
		return X509_STATUS_FORMAT;
	}
	while (sequence.Value.Len != 0U)
	{
		if (!X509DerNext(&sequence.Value, &item))
		{
			return X509_STATUS_FORMAT;
		}
		index = (item.Tag & 0x1fU) + 1U;
		if (index <= previous)
		{
			return X509_STATUS_FORMAT;
		}
		previous = index;
		if (item.Tag == 0x80U)
		{
			if (item.Value.Len == 0U)
			{
				return X509_STATUS_FORMAT;
			}
		}
		else if (item.Tag == 0xa1U)
		{
			issuer = true;
			/* authorityCertIssuer commonly uses directoryName. It is not used
			 * to build a path: the supplied ordered names and signatures are. */
			if (item.Value.Len == 0U)
			{
				return X509_STATUS_FORMAT;
			}
			while (item.Value.Len != 0U)
			{
				if (!X509DerNext(&item.Value, &generalName))
				{
					return X509_STATUS_FORMAT;
				}
				if (generalName.Tag == 0xa4U)
				{
					if (!X509Name(&generalName.Value, &name, &empty) ||
						generalName.Value.Len != 0U || empty)
					{
						return X509_STATUS_FORMAT;
					}
				}
				else
				{
					status = X509GeneralNames(generalName.Raw, noHost, &match);
					if (status != X509_STATUS_OK)
					{
						return status;
					}
				}
			}
		}
		else if (item.Tag == 0x82U)
		{
			serial = true;
			if (!X509PositiveInt(item.Value) || item.Value.Len > 20U ||
				(item.Value.Len == 1U && item.Value.pData[0] == 0U))
			{
				return X509_STATUS_FORMAT;
			}
		}
		else
		{
			return X509_STATUS_FORMAT;
		}
	}
	return issuer == serial ? X509_STATUS_OK : X509_STATUS_FORMAT;
}

static X509_STATUS X509ExtensionValue(X509Span_t Oid, X509Span_t Value,
									bool bCritical, X509Span_t Host,
									X509Info_t *pInfo)
{
	X509Der_t item;

	if (Oid.Len == 3U && Oid.pData[0] == 0x55U && Oid.pData[1] == 0x1dU)
	{
		switch (Oid.pData[2])
		{
			case 14U: /* subjectKeyIdentifier */
				if (!X509DerRead(&Value, 0x04U, &item) || Value.Len != 0U ||
					item.Value.Len == 0U)
				{
					return X509_STATUS_FORMAT;
				}
				return bCritical ? X509_STATUS_UNSUPPORTED : X509_STATUS_OK;
			case 15U:
				return X509KeyUsage(Value, pInfo);
			case 17U:
				return X509SubjectAltName(Value, Host, bCritical, pInfo);
			case 19U:
				return X509BasicConstraints(Value, bCritical, pInfo);
			case 30U: /* nameConstraints */
			case 33U: /* policyMappings */
			case 36U: /* policyConstraints */
			case 54U: /* inhibitAnyPolicy */
				return X509_STATUS_UNSUPPORTED;
			case 35U: /* authorityKeyIdentifier */
				return bCritical ? X509_STATUS_UNSUPPORTED :
					X509AuthorityKeyId(Value);
			case 37U:
				return X509ExtendedKeyUsage(Value, pInfo);
			default:
				break;
		}
	}
	return bCritical ? X509_STATUS_UNSUPPORTED : X509_STATUS_OK;
}

static X509_STATUS X509Extensions(X509Span_t *pInput, X509Span_t Host,
								 X509Info_t *pInfo)
{
	X509Der_t explicitValue;
	X509Der_t sequence;
	X509Der_t extension;
	X509Der_t oid;
	X509Der_t critical;
	X509Der_t value;
	X509Span_t seen[X509_MAX_EXTENSIONS];
	uint32_t count = 0U;
	uint32_t i;
	bool bCritical;
	X509_STATUS status;

	if (!X509DerRead(pInput, 0xa3U, &explicitValue) ||
		!X509DerRead(&explicitValue.Value, 0x30U, &sequence) ||
		explicitValue.Value.Len != 0U || sequence.Value.Len == 0U)
	{
		return X509_STATUS_FORMAT;
	}
	while (sequence.Value.Len != 0U)
	{
		if (count == X509_MAX_EXTENSIONS)
		{
			return X509_STATUS_LIMIT;
		}
		if (!X509DerRead(&sequence.Value, 0x30U, &extension) ||
			!X509DerRead(&extension.Value, 0x06U, &oid) ||
			!X509OidValid(oid.Value))
		{
			return X509_STATUS_FORMAT;
		}
		for (i = 0U; i < count; i++)
		{
			if (X509Equal(oid.Value, seen[i]))
			{
				return X509_STATUS_FORMAT;
			}
		}
		seen[count++] = oid.Value;
		bCritical = false;
		if (extension.Value.Len != 0U && extension.Value.pData[0] == 0x01U)
		{
			if (!X509DerRead(&extension.Value, 0x01U, &critical) ||
				critical.Value.Len != 1U || critical.Value.pData[0] != 0xffU)
			{
				return X509_STATUS_FORMAT;
			}
			bCritical = true;
		}
		if (!X509DerRead(&extension.Value, 0x04U, &value) ||
			extension.Value.Len != 0U)
		{
			return X509_STATUS_FORMAT;
		}
		status = X509ExtensionValue(oid.Value, value.Value, bCritical, Host, pInfo);
		if (status != X509_STATUS_OK)
		{
			return status;
		}
	}
	return X509_STATUS_OK;
}

static X509_STATUS X509Parse(const X509Cert_t *pCert, X509Span_t Host,
							X509Info_t *pInfo)
{
	X509Span_t input;
	X509Der_t certificate;
	X509Der_t tbs;
	X509Der_t item;
	X509Der_t version;
	X509Der_t validity;
	uint8_t signature[64];
	bool issuerEmpty;
	X509_STATUS status;

	memset(pInfo, 0, sizeof(*pInfo));
	if (pCert->pDer == nullptr || pCert->Len == 0U)
	{
		return X509_STATUS_ARG;
	}
	if (pCert->Len > X509_MAX_CERT_SIZE)
	{
		return X509_STATUS_LIMIT;
	}
	input.pData = pCert->pDer;
	input.Len = pCert->Len;
	if (!X509DerRead(&input, 0x30U, &certificate) || input.Len != 0U ||
		!X509DerRead(&certificate.Value, 0x30U, &tbs))
	{
		return X509_STATUS_FORMAT;
	}
	pInfo->Tbs = tbs.Raw;
	status = X509SignatureAlgorithm(&certificate.Value);
	if (status != X509_STATUS_OK)
	{
		return status;
	}
	if (!X509DerRead(&certificate.Value, 0x03U, &item) ||
		certificate.Value.Len != 0U || item.Value.Len < 2U ||
		item.Value.pData[0] != 0U ||
		!X509EcdsaP256Decode(item.Value.pData + 1U, item.Value.Len - 1U, signature))
	{
		return X509_STATUS_FORMAT;
	}
	pInfo->Signature.pData = item.Value.pData + 1U;
	pInfo->Signature.Len = item.Value.Len - 1U;
	if (!X509DerRead(&tbs.Value, 0xa0U, &item) ||
		!X509DerRead(&item.Value, 0x02U, &version) || item.Value.Len != 0U ||
		version.Value.Len != 1U || version.Value.pData[0] != 2U)
	{
		return X509_STATUS_FORMAT;
	}
	if (!X509DerRead(&tbs.Value, 0x02U, &item) ||
		!X509PositiveInt(item.Value) || item.Value.Len > 20U ||
		(item.Value.Len == 1U && item.Value.pData[0] == 0U))
	{
		return X509_STATUS_FORMAT;
	}
	status = X509SignatureAlgorithm(&tbs.Value);
	if (status != X509_STATUS_OK)
	{
		return status;
	}
	if (!X509Name(&tbs.Value, &pInfo->Issuer, &issuerEmpty) || issuerEmpty ||
		!X509DerRead(&tbs.Value, 0x30U, &validity) ||
		!X509Time(&validity.Value, &pInfo->NotBefore) ||
		!X509Time(&validity.Value, &pInfo->NotAfter) || validity.Value.Len != 0U ||
		pInfo->NotBefore > pInfo->NotAfter ||
		!X509Name(&tbs.Value, &pInfo->Subject, &pInfo->bSubjectEmpty))
	{
		return X509_STATUS_FORMAT;
	}
	status = X509PublicKey(&tbs.Value, &pInfo->pPublicKey);
	if (status != X509_STATUS_OK)
	{
		return status;
	}
	if (tbs.Value.Len != 0U)
	{
		/* issuerUniqueID and subjectUniqueID are outside this profile. */
		if (tbs.Value.pData[0] == 0x81U || tbs.Value.pData[0] == 0x82U)
		{
			return X509_STATUS_UNSUPPORTED;
		}
		status = X509Extensions(&tbs.Value, Host, pInfo);
		if (status != X509_STATUS_OK)
		{
			return status;
		}
	}
	if (tbs.Value.Len != 0U)
	{
		return X509_STATUS_FORMAT;
	}
	if ((pInfo->bSubjectEmpty && (!pInfo->bSan || !pInfo->bSanCritical)) ||
		(!pInfo->bCa && (pInfo->Usage & X509_KEY_CERT_SIGN) != 0U) ||
		(pInfo->bPathLen && (!pInfo->bUsage ||
		(pInfo->Usage & X509_KEY_CERT_SIGN) == 0U)))
	{
		return X509_STATUS_CONSTRAINT;
	}
	return X509_STATUS_OK;
}

static X509_STATUS X509CheckTime(const X509Info_t *pInfo, uint64_t UnixTime)
{
	return (int64_t)UnixTime >= pInfo->NotBefore &&
		(int64_t)UnixTime <= pInfo->NotAfter ? X509_STATUS_OK : X509_STATUS_TIME;
}

static X509_STATUS X509CheckCa(const X509Info_t *pInfo, uint32_t CaBelow)
{
	if (!pInfo->bBasic || !pInfo->bBasicCritical || !pInfo->bCa ||
		pInfo->bSubjectEmpty ||
		(pInfo->bUsage && (pInfo->Usage & X509_KEY_CERT_SIGN) == 0U) ||
		(pInfo->bEku && !pInfo->bServerAuth) ||
		(pInfo->bPathLen && CaBelow > pInfo->PathLen))
	{
		return X509_STATUS_CONSTRAINT;
	}
	return X509_STATUS_OK;
}

static X509_STATUS X509CheckSignature(const X509VerifyCfg_t *pCfg,
									const X509Info_t *pChild,
									const X509Info_t *pIssuer)
{
	uint8_t digest[32];
	uint8_t signature[64];
	CRYPTO_STATUS status;

	if (!X509Equal(pChild->Issuer, pIssuer->Subject))
	{
		return X509_STATUS_UNTRUSTED;
	}
	if (!X509EcdsaP256Decode(pChild->Signature.pData, pChild->Signature.Len,
			signature))
	{
		return X509_STATUS_FORMAT;
	}
	status = pCfg->pHash->Hash(CRYPTO_HASH_SHA256, pChild->Tbs.pData,
		pChild->Tbs.Len, digest);
	if (status != CRYPTO_STATUS_OK)
	{
		return X509_STATUS_CRYPTO;
	}
	status = pCfg->pSign->Verify(CRYPTO_CURVE_P256, pIssuer->pPublicKey,
		digest, sizeof(digest), signature);
	if (status == CRYPTO_STATUS_OK)
	{
		return X509_STATUS_OK;
	}
	return status == CRYPTO_STATUS_FAIL ? X509_STATUS_SIGNATURE : X509_STATUS_CRYPTO;
}

X509_STATUS X509VerifyChain(const X509VerifyCfg_t *pCfg,
						 const X509Cert_t *pPeerCerts, uint32_t PeerCount,
						 uint8_t pLeafKey[64])
{
	X509Info_t peers[X509_MAX_CHAIN_CERTS];
	X509Info_t anchor;
	X509Span_t host;
	X509Span_t noHost = { nullptr, 0U };
	X509Span_t peerDer;
	X509Span_t anchorDer;
	uint32_t i;
	uint32_t caBelow = 0U;
	uint32_t anchorBelow;
	bool same;
	X509_STATUS status;
	X509_STATUS result = X509_STATUS_UNTRUSTED;

	if (pLeafKey == nullptr)
	{
		return X509_STATUS_ARG;
	}
	memset(pLeafKey, 0, 64U);
	if (pCfg == nullptr || pPeerCerts == nullptr || PeerCount == 0U ||
		pCfg->pHash == nullptr || pCfg->pSign == nullptr ||
		pCfg->pTrustAnchors == nullptr || pCfg->TrustAnchorCount == 0U ||
		pCfg->pHostName == nullptr || pCfg->UnixTime == 0U ||
		pCfg->UnixTime > 253402300799ULL)
	{
		return X509_STATUS_ARG;
	}
	if (PeerCount > X509_MAX_CHAIN_CERTS ||
		pCfg->TrustAnchorCount > X509_MAX_TRUST_ANCHORS)
	{
		return X509_STATUS_LIMIT;
	}
	if (pCfg->pHash->IsAsync() || pCfg->pSign->IsAsync())
	{
		return X509_STATUS_CRYPTO;
	}
	host.pData = (const uint8_t *)pCfg->pHostName;
	host.Len = 0U;
	while (host.Len <= X509_MAX_HOSTNAME_SIZE && host.pData[host.Len] != 0U)
	{
		host.Len++;
	}
	if (!X509DnsValid(host, false))
	{
		return X509_STATUS_HOSTNAME;
	}
	for (i = 0U; i < PeerCount; i++)
	{
		status = X509Parse(pPeerCerts + i, i == 0U ? host : noHost, peers + i);
		if (status != X509_STATUS_OK)
		{
			return status;
		}
		status = X509CheckTime(peers + i, pCfg->UnixTime);
		if (status != X509_STATUS_OK)
		{
			return status;
		}
		if (i == 0U)
		{
			if (peers[i].bCa ||
				(peers[i].bUsage &&
				(peers[i].Usage & X509_KEY_DIGITAL_SIGNATURE) == 0U) ||
				(peers[i].bEku && !peers[i].bServerAuth))
			{
				return X509_STATUS_CONSTRAINT;
			}
			if (!peers[i].bSan || !peers[i].bHostMatch)
			{
				return X509_STATUS_HOSTNAME;
			}
		}
		else
		{
			status = X509CheckCa(peers + i, caBelow);
			if (status == X509_STATUS_OK)
			{
				status = X509CheckSignature(pCfg, peers + i - 1U, peers + i);
			}
			if (status != X509_STATUS_OK)
			{
				return status;
			}
			if (!X509Equal(peers[i].Issuer, peers[i].Subject))
			{
				caBelow++;
			}
		}
	}
	peerDer.pData = pPeerCerts[PeerCount - 1U].pDer;
	peerDer.Len = pPeerCerts[PeerCount - 1U].Len;
	for (i = 0U; i < pCfg->TrustAnchorCount; i++)
	{
		status = X509Parse(pCfg->pTrustAnchors + i, noHost, &anchor);
		if (status != X509_STATUS_OK)
		{
			return status;
		}
		anchorDer.pData = pCfg->pTrustAnchors[i].pDer;
		anchorDer.Len = pCfg->pTrustAnchors[i].Len;
		same = X509Equal(peerDer, anchorDer);
		if (!same && !X509Equal(peers[PeerCount - 1U].Issuer, anchor.Subject))
		{
			continue;
		}
		anchorBelow = caBelow;
		if (same && PeerCount > 1U &&
			!X509Equal(anchor.Issuer, anchor.Subject))
		{
			anchorBelow--;
		}
		status = X509CheckTime(&anchor, pCfg->UnixTime);
		if (status == X509_STATUS_OK)
		{
			status = X509CheckCa(&anchor, anchorBelow);
		}
		if (status == X509_STATUS_OK && !same)
		{
			status = X509CheckSignature(pCfg, peers + PeerCount - 1U, &anchor);
		}
		if (status == X509_STATUS_OK)
		{
			memcpy(pLeafKey, peers[0].pPublicKey, 64U);
			return X509_STATUS_OK;
		}
		result = status;
	}
	return result;
}
