
#ifndef __LPC546XX_H__
#define __LPC546XX_H__

#ifdef LPC54605_SERIES
#include "LPC54605.h"
#include "LPC54605_features.h"
#elif defined(LPC54606_SERIES)
#include "LPC54606.h"
#include "LPC54606_features.h"
#elif defined(LPC54607_SERIES)
#include "LPC54607.h"
#include "LPC54607_features.h"
#elif defined(LPC54608_SERIES)
#include "LPC54608.h"
#include "LPC54608_features.h"
#elif defined(LPC54616_SERIES)
#include "LPC54616.h"
#include "LPC54616_features.h"
#elif defined(LPC54618_SERIES)
#include "LPC54618.h"
#include "LPC54618_features.h"
#elif defined(LPC54628_SERIES)
#include "LPC54628.h"
#include "LPC54628_features.h"
#else
#error "No valid CPU defined!"
#endif

#endif  // __LPC546XX_H__

