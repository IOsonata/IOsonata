from pathlib import Path

path = Path("tests/bluetooth/host/bt_gatt_adversarial_test.cpp")
text = path.read_text()
old = '''\tif (!registered)\n\t{\n\t\tstd::memset(chars, 0, sizeof(chars));\n\t\tstd::memset(&service, 0, sizeof(service));\n\t\tfor (int i = 0; i < 2; i++)\n\t\t{\n\t\t\tchars[i].Uuid = (uint16_t)(0xFFC0 + i);\n\t\t\tchars[i].MaxDataLen = 16;\n\t\t\tchars[i].Property = BT_GATT_CHAR_PROP_READ |\n\t\t\t\t\t\t\t BT_GATT_CHAR_PROP_WRITE;\n\t\t}\n\t\tchars[0].WrCB = LongWriteChanged;\n\t\tservice.UuidSrvc = 0xFFBF;\n\t\tservice.NbChar = 2;\n\t\tservice.pCharArray = chars;\n\t\tBT_CHECK(s_Test, BtGattSrvcAdd(&service));\n\t\tregistered = true;\n\t}\n\n\ts_Peer.Conn.pLongWrBuff = queue;'''
new = '''\tif (!registered)\n\t{\n\t\tstd::memset(chars, 0, sizeof(chars));\n\t\tstd::memset(&service, 0, sizeof(service));\n\t\tfor (int i = 0; i < 2; i++)\n\t\t{\n\t\t\tchars[i].Uuid = (uint16_t)(0xFFC0 + i);\n\t\t\tchars[i].MaxDataLen = 16;\n\t\t\tchars[i].Property = BT_GATT_CHAR_PROP_READ |\n\t\t\t\t\t\t\t BT_GATT_CHAR_PROP_WRITE;\n\t\t}\n\t\tchars[0].WrCB = LongWriteChanged;\n\t\tservice.UuidSrvc = 0xFFBF;\n\t\tservice.NbChar = 2;\n\t\tservice.pCharArray = chars;\n\t\tBT_CHECK(s_Test, BtGattSrvcAdd(&service));\n\t\tregistered = true;\n\t}\n\n\tBT_CHECK(s_Test, BtGattCharSetValue(&chars[0], nullptr, 0));\n\tBT_CHECK(s_Test, BtGattCharSetValue(&chars[1], nullptr, 0));\n\n\ts_Peer.Conn.pLongWrBuff = queue;'''
if text.count(old) != 1:
    raise RuntimeError(f"expected test setup once, found {text.count(old)}")
text = text.replace(old, new, 1)
text = text.replace(
    "BT_CHECK(s_Test, len == 5 + f.Len);",
    "BT_CHECK(s_Test, len == (uint32_t)(5 + f.Len));",
    1,
)
path.write_text(text)
print("final callback test starting state adjusted")
