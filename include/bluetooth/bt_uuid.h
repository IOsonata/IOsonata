/**-------------------------------------------------------------------------
@file	bt_uuid.h

@brief	Bluetooth standard uuid definitions

Generic implementation & definitions of Bluetooth UUID. Contains most of the
standard Bluetooth UUID values and utility management functions.
Maintain a table of stored base 128 bits UUID. The table size is defined by
BT_BASE_UUID_ENTRY_MAX_COUNT at compile time. Add this define at the compiler
preprocessor option to overwrite the default value when compiling the IOsonata library.
This has no effect if added to application firmware project setting, only for compiling
the IOsonata library.


@author	Hoang Nguyen Hoan
@date	Mar. 8, 2022

@license

MIT License

Copyright (c) 2022, I-SYST, all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#ifndef __BT_UUID_H__
#define __BT_UUID_H__

#include <stdint.h>

/** @addtogroup Bluetooth
 * @{ */

// 3.1 Protocol Identifiers
#define BT_UUID_PROTOCOL_SDP                                        				0x0001	// Bluetooth Core Specs
#define BT_UUID_PROTOCOL_UDP                                        				0x0002	// Personal Area Networking Profile
#define BT_UUID_PROTOCOL_RFCOMM                                     				0x0003	// RFCOMM
#define BT_UUID_PROTOCOL_TCP                                        				0x0004	// Personal Area Networking Profile
#define BT_UUID_PROTOCOL_TCS_BIN                                    				0x0005	// Telephony Control Protocol
#define BT_UUID_PROTOCOL_TCS_AT                                     				0x0006	// Telephony Control Protocol
#define BT_UUID_PROTOCOL_ATT                                        				0x0007	// Bluetooth Core Spec
#define BT_UUID_PROTOCOL_OBEX                                       				0x0008	// IrDA Interoperability
#define BT_UUID_PROTOCOL_IP                                         				0x0009	//
#define BT_UUID_PROTOCOL_FTP                                        				0x000A
#define BT_UUID_PROTOCOL_HTTP                                       				0x000C
#define BT_UUID_PROTOCOL_WSP                                        				0x000E	// WAP Bearer
#define BT_UUID_PROTOCOL_BNEP                                       				0x000F	// Bluetooth Network Encapsulation Protocol
#define BT_UUID_PROTOCOL_UPNP                                       				0x0010	// Extended Service Discovery Profile for UPnP
#define BT_UUID_PROTOCOL_HIDP                                       				0x0011	// Human Interface Device Profile
#define BT_UUID_PROTOCOL_HARDCOPY_CONTROL_CHANNEL                   				0x0012	// Hardcopy Cable Replacement Profile
#define BT_UUID_PROTOCOL_HARDCOPY_DATA_CHANNEL                      				0x0014	// Hardcopy Cable Replacement Profile
#define BT_UUID_PROTOCOL_HARDCOPY_NOTIFICATION                      				0x0016
#define BT_UUID_PROTOCOL_AVCTP                                      				0x0017	// Audio/Video Control Transport Protocol
#define BT_UUID_PROTOCOL_AVDTP                                      				0x0019	// Audio/Video Distribution Transport Protocol
#define BT_UUID_PROTOCOL_CMTP                                       				0x001B	// Common ISDN Access Profile
#define BT_UUID_PROTOCOL_MCAP_CONTROL_CHANNEL                       				0x001E	// Health Device Profile
#define BT_UUID_PROTOCOL_MCAP_DATA_CHANNEL                          				0x001F
#define BT_UUID_PROTOCOL_L2CAP                              						0x0100	// Bluetooth Core Spec

// 3.2 Browse Group Identifiers
#define BT_UUID_BROWSE_PUBLIC_ROOT													0x1002

// 3.2 SDP Service Class and Profile Identifiers
#define BT_UUID_SDP_SERVICE_DISCOVERY_SERVER_SERVICE_CLASS_ID 						0x1000
#define BT_UUID_SDP_BROWSER_GROUP_DESC_SERVICE_CLASS_ID   							0x1001
#define BT_UUID_SDP_SERIAL_PORT                           							0x1101
#define BT_UUID_SDP_LAN_ACCESS_USING_PPP                  							0x1102
#define BT_UUID_SDP_DIALUP_NETWORKING                     							0x1103
#define BT_UUID_SDP_IR_MC_SYNC                            							0x1104
#define BT_UUID_SDP_OBEX_OBJECT_PUSH                      							0x1105
#define BT_UUID_SDP_OBEX_FILE_TRANSFER                    							0x1106
#define BT_UUID_SDP_IR_MC_SYNC_COMMAND                    							0x1107
#define BT_UUID_SDP_HEADSET                               							0x1108
#define BT_UUID_SDP_CORDLESS_TELEPHONY                    							0x1109
#define BT_UUID_SDP_AUDIO_SOURCE                          							0x110A
#define BT_UUID_SDP_AUDIO_SINK                            							0x110B
#define BT_UUID_SDP_AV_REMOTE_CONTROL_TARGET              							0x110C
#define BT_UUID_SDP_ADVANCED_AUDIO_DISTRIBUTION           							0x110D
#define BT_UUID_SDP_AV_REMOTE_CONTROL                     							0x110E
#define BT_UUID_SDP_AV_REMOTE_CONTROL_CONTROLLER          							0x110F
#define BT_UUID_SDP_INTERCOM                              							0x1110
#define BT_UUID_SDP_FAX                                   							0x1111
#define BT_UUID_SDP_HEADSET_AUDIO_GATEWAY                 							0x1112
#define BT_UUID_SDP_WAP                                   							0x1113
#define BT_UUID_SDP_WAP_CLIENT                            							0x1114
#define BT_UUID_SDP_PANU                                  							0x1115
#define BT_UUID_SDP_NAP                                   							0x1116
#define BT_UUID_SDP_GN                                    							0x1117
#define BT_UUID_SDP_DIRECT_PRINTING                       							0x1118
#define BT_UUID_SDP_REFERENCE_PRINTING                    							0x1119
#define BT_UUID_SDP_BASIC_IMAGING_PROFILE                 							0x111A
#define BT_UUID_SDP_IMAGING_RESPONDER                     							0x111B
#define BT_UUID_SDP_IMAGING_AUTOMATIC_ARCHIVE             							0x111C
#define BT_UUID_SDP_IMAGING_REFERENC_OBJECTS              							0x111D
#define BT_UUID_SDP_HANDFREE                              							0x111E
#define BT_UUID_SDP_HANDFREE_AUDIO_GATEWAY                							0x111F
#define BT_UUID_SDP_DIRECT_PRINTING_REFERENCE_OBJECTS_SErVICE						0x1120
#define BT_UUID_SDP_REFLECTED_UI                          							0x1121
#define BT_UUID_SDP_BASIC_PRINTING                        							0x1122
#define BT_UUID_SDP_PRINTING_STATUS                       							0x1123
#define BT_UUID_SDP_HUMAN_INTERFACE_DEVICE_SERVICE        							0x1124  // HID service
#define BT_UUID_SDP_HARDCOPY_CABLE_REPLACEMENT            							0x1125
#define BT_UUID_SDP_HCR_PRINT                             							0x1126
#define BT_UUID_SDP_HCR_SCAN                              							0x1127
#define BT_UUID_SDP_COMMON_ISDN_ACCESS                    							0x1128
#define BT_UUID_SDP_SIM_ACCESS                            							0x112D
#define BT_UUID_SDP_PHONEBOOK_ACCESS_PCE                  							0x112E
#define BT_UUID_SDP_PHONEBOOK_ACCESS_PSE                  							0x112F
#define BT_UUID_SDP_PHONEBOOK_ACCESS                      							0x1130
#define BT_UUID_SDP_HEADSET_HS                            							0x1131
#define BT_UUID_SDP_MESSAGE_ACCESS_SERVER                 							0x1132
#define BT_UUID_SDP_MESSAGE_NOTIFICATION_SErVER           							0x1133
#define BT_UUID_SDP_MESSAGE_ACCESS_PROFILE                							0x1134
#define BT_UUID_SDP_GNSS                                  							0x1135
#define BT_UUID_SDP_GNSS_SERVER                           							0x1136
#define BT_UUID_SDP_3D_DISPLAY                            							0x1137
#define BT_UUID_SDP_3D_GLASSES                            							0x1138
#define BT_UUID_SDP_3D_SYNCHRONIZATION                    							0x1139
#define BT_UUID_SDP_MPS_PROFILE_UUID                      							0x113A
#define BT_UUID_SDP_MPS_SC_UUID                           							0x113B
#define BT_UUID_SDP_CTN_ACCESS_SERVICE                    							0x113C
#define BT_UUID_SDP_CTN_NOTIFICATION_SERVICE              							0x113D
#define BT_UUID_SDP_CTN_PROFILE                           							0x113E
#define BT_UUID_SDP_PNP_INFORMATION                       							0x1200
#define BT_UUID_SDP_GENERIC_NETWORKING                    							0x1201
#define BT_UUID_SDP_GENERIC_FILE_TRANSFER                 							0x1202
#define BT_UUID_SDP_GENERIC_AUDIO                         							0x1203
#define BT_UUID_SDP_GENERIC_TELEPHONY                     							0x1204
#define BT_UUID_SDP_UPNP_SERVICE                          							0x1205
#define BT_UUID_SDP_UPNP_IP_SERVICE                       							0x1206
#define BT_UUID_SDP_ESDP_UPNP_IP_PAN                      							0x1300
#define BT_UUID_SDP_ESDP_UPNP_IP_LAP                      							0x1301
#define BT_UUID_SDP_ESDP_UPNP_L2CAP                       							0x1302
#define BT_UUID_SDP_VIDEO_SOURCE                          							0x1303
#define BT_UUID_SDP_VIDEO_SINK                            							0x1304
#define BT_UUID_SDP_VIDEO_DISTRIBUTION                    							0x1305
#define BT_UUID_SDP_HDP                                   							0x1400
#define BT_UUID_SDP_HDP_SOURCE                            							0x1401
#define BT_UUID_SDP_HDP_SINK                              							0x1402

// 3.4 GATT Services
#define BT_UUID_GATT_SERVICE_GENERIC_ACCESS                         				0x1800
#define BT_UUID_GATT_SERVICE_GENERIC_ATTRIBUTE                      				0x1801
#define BT_UUID_GATT_SERVICE_IMMEDIATE_ALERT                        				0x1802
#define BT_UUID_GATT_SERVICE_LINK_LOSS                              				0x1803
#define BT_UUID_GATT_SERVICE_TX_POWER                               				0x1804
#define BT_UUID_GATT_SERVICE_CURRENT_TIME                           				0x1805
#define BT_UUID_GATT_SERVICE_REFERENCE_TIME_UPDATE                  				0x1806
#define BT_UUID_GATT_SERVICE_NEXT_DST_CHANGE                        				0x1807
#define BT_UUID_GATT_SERVICE_GLUCOSE                                				0x1808
#define BT_UUID_GATT_SERVICE_HEALTH_THERMOMETER                     				0x1809
#define BT_UUID_GATT_SERVICE_DEVICE_INFORMATION                     				0x180A
#define BT_UUID_GATT_SERVICE_HEART_RATE                             				0x180D
#define BT_UUID_GATT_SERVICE_PHONE_ALERT_STATUS                     				0x180E
#define BT_UUID_GATT_SERVICE_BATTERY                                				0x180F
#define BT_UUID_GATT_SERVICE_BLOOD_PRESSURE                         				0x1810
#define BT_UUID_GATT_SERVICE_ALERT_NOTIFICATION                     				0x1811
#define BT_UUID_GATT_SERVICE_HUMAN_INTERFACE_DEVICE                 				0x1812  // HID
#define BT_UUID_GATT_SERVICE_SCAN_PARAMETERS                        				0x1813
#define BT_UUID_GATT_SERVICE_RUNNING_SPEED_AND_CADENCE              				0x1814
#define BT_UUID_GATT_SERVICE_AUTOMATION_IO                          				0x1815
#define BT_UUID_GATT_SERVICE_CYCLING_SPEED_AND_CADENCE              				0x1816
#define BT_UUID_GATT_SERVICE_CYCLING_POWER                          				0x1818
#define BT_UUID_GATT_SERVICE_LOCATION_AND_NAVIGATION                				0x1819
#define BT_UUID_GATT_SERVICE_ENVIRONMENTAL_SENSING                  				0x181A
#define BT_UUID_GATT_SERVICE_BODY_COMPOSITION                       				0x181B
#define BT_UUID_GATT_SERVICE_USER_DATA                              				0x181C
#define BT_UUID_GATT_SERVICE_WEIGHT_SCALE                           				0x181D
#define BT_UUID_GATT_SERVICE_BOND_MANAGEMENT                        				0x181E
#define BT_UUID_GATT_SERVICE_CONTINUOUS_GLUCOSE_MONITORING          				0x181F
#define BT_UUID_GATT_SERVICE_INTERNET_PROTOCOL_SUPPORT              				0x1820
#define BT_UUID_GATT_SERVICE_INDOOR_POSITIONING                     				0x1821
#define BT_UUID_GATT_SERVICE_PULSE_OXIMETER                         				0x1822
#define BT_UUID_GATT_SERVICE_HTTP_PROXY                             				0x1823
#define BT_UUID_GATT_SERVICE_TRANSPORT_DISCOVERY                    				0x1824
#define BT_UUID_GATT_SERVICE_OBJECT_TRANSFER                        				0x1825
#define BT_UUID_GATT_SERVICE_FITNESS_MACHINE                        				0x1826
#define BT_UUID_GATT_SERVICE_MESH_PROVISIONING                      				0x1827
#define BT_UUID_GATT_SERVICE_MESH_PROXY                             				0x1828
#define BT_UUID_GATT_SERVICE_RECONNECTION_CONFIGURATION             				0x1829
#define BT_UUID_GATT_SERVICE_INSULIN_DELIVERY                       				0x183A
#define BT_UUID_GATT_SERVICE_BINARY_SENSOR                          				0x183B
#define BT_UUID_GATT_SERVICE_EMERGENCY_CONFIGURATION                				0x183C
#define BT_UUID_GATT_SERVICE_PHYSICAL_ACTIVITY_MONITOR              				0x183E
#define BT_UUID_GATT_SERVICE_ELAPSE_TIME											0x183F
#define BT_UUID_GATT_SERVICE_GENERIC_HEALTH_SENSOR									0x1840
#define BT_UUID_GATT_SERVICE_AUDIO_INPUT_CONTROL                    				0x1843
#define BT_UUID_GATT_SERVICE_VOLUME_CONTROL                         				0x1844
#define BT_UUID_GATT_SERVICE_VOLUME_OFFSET_CONTROL                  				0x1845
#define BT_UUID_GATT_SERVICE_COORDINATED_SET_IDENTIFICATION_SERVICE 				0x1846
#define BT_UUID_GATT_SERVICE_DEVICE_TIME                            				0x1847
#define BT_UUID_GATT_SERVICE_MEDIA_CONTROL_SERVICE                  				0x1848
#define BT_UUID_GATT_SERVICE_GENERIC_MEDIA_CONTROL_SERVICE          				0x1849
#define BT_UUID_GATT_SERVICE_CONSTANT_TONE_EXTENSION                				0x184A
#define BT_UUID_GATT_SERVICE_TELEPHONE_BEARER_SERVICE               				0x184B
#define BT_UUID_GATT_SERVICE_GENERIC_TELEPHONE_BEARER_SERVICE       				0x184C
#define BT_UUID_GATT_SERVICE_MICROPHONE_CONTROL                     				0x184D
#define BT_UUID_GATT_SERVICE_AUDIO_STREAM_CONTROL_SERVICE           				0x184E
#define BT_UUID_GATT_SERVICE_BROADCAST_AUTIO_SCAN_SERVICE           				0x184F
#define BT_UUID_GATT_SERVICE_PUBLISHED_AUDIO_CAPABILITIES_SERVICE   				0x1850
#define BT_UUID_GATT_SERVICE_BASIC_AUDIO_ANNOUNCEMENT_SERVICE       				0x1851
#define BT_UUID_GATT_SERVICE_BORADCAST_AUDIO_ANNOUNCEMENT_SERVICE   				0x1852
#define BT_UUID_GATT_SERVICE_COMMON_AUDIO_SERVICE                   				0x1853  // Could be changed
#define BT_UUID_GATT_SERVICE_HEARING_ACCESS_SERVICE                 				0x1854  // Could be changed
#define BT_UUID_GATT_SERVICE_TMAS_SERVICE                           				0x1855  // Could be changed
#define BT_UUID_GATT_SERVICE_PUBLIC_BROADCAST_ANNOUNCEMENT							0x1856
#define BT_UUID_GATT_SERVICE_ELECTRONIC_SHELF_LABEL									0x1857
#define BT_UUID_GATT_SERVICE_GAMING_AUDIO											0x1858
#define BT_UUID_GATT_SERVICE_MESH_PROXY_SOLICITATION								0x1859

// 3.5 Units
#define BT_UUID_UNIT_UNITLESS                                  						0x2700
#define BT_UUID_UNIT_LENGTH_METRE                              						0x2701
#define BT_UUID_UNIT_MASS_KG                                   						0x2702
#define BT_UUID_UNIT_TIME_SECOND                               						0x2703
#define BT_UUID_UNIT_ELECTRIC_CURRENT_AMPERE                   						0x2704
#define BT_UUID_UNIT_THERMODYNAMIC_TEMPERATURE_KELVIN          						0x2705
#define BT_UUID_UNIT_AMOUNT_OF_SUBSTANCE_MOLE                  						0x2706
#define BT_UUID_UNIT_LUMINOUS_INTENSITY_CANDELA                						0x2707
#define BT_UUID_UNIT_AREA_SQUARE_METRES                        						0x2710
#define BT_UUID_UNIT_VOLUME_CUBIC_METERS                       						0x2711
#define BT_UUID_UNIT_VELOCITY_METRE_PER_SECOND                 						0x2712
#define BT_UUID_UNIT_ACCELERATION_METRE_PER_SECOND_SQUARED     						0x2713
#define BT_UUID_UNIT_WAVENUMBER_RECIPROCAL_METRE               						0x2714
#define BT_UUID_UNIT_DENSITY_KG_PER_CUBIC_METRE                						0x2715
#define BT_UUID_UNIT_SURFACE_DENSITY_KG_PER_SQUARE_METRE       						0x2716
#define BT_UUID_UNIT_SPECIFIC_VOLUME_CUBIC_METRE_PER_KG        						0x2717
#define BT_UUID_UNIT_CURRENT_DENSITY_AMPERE_PER_SQUARE_METRE   						0x2718
#define BT_UUID_UNIT_MAGNETIC_FIELD_STRENGTH_AMPERE_PER_METRE  						0x2719
#define BT_UUID_UNIT_AMOUNT_CONCENTRATION_MOLE_PER_CUBIC_METRE 						0x271A
#define BT_UUID_UNIT_MASS_CONCENTRATION_KG_PER_CUBIC_METRE     						0x271B
#define BT_UUID_UNIT_LIMINACE_CANDELA_PER_SQUARE_METRE         						0x271C
#define BT_UUID_UNIT_REFRACTIVE_INDEX                          						0x271D
#define BT_UUID_UNIT_RELATIVE_PERMEABILITY                     						0x271E
#define BT_UUID_UNIT_PLANE_ANGLE_RADIAN                        						0x2720
#define BT_UUID_UNIT_SOLID_ANGLE_STERADIAN                     						0x2721
#define BT_UUID_UNIT_FREQUENCY_HERTZ                           						0x2722
#define BT_UUID_UNIT_FORCE_NEWTON                              						0x2723
#define BT_UUID_UNIT_PRESSURE_PASCAL                           						0x2724
#define BT_UUID_UNIT_ENERGY_JOULE                              						0x2725
#define BT_UUID_UNIT_POWER_WATT                                						0x2726
#define BT_UUID_UNIT_ELECTRIC_CHARGE_COULOMB                   						0x2727
#define BT_UUID_UNIT_ELECTRIC_POTENTIAL_DIFFERENCE_VOLT        						0x2728
#define BT_UUID_UNIT_CAPACITANCE_FARAD                         						0x2729
#define BT_UUID_UNIT_ELECTRIC_RESISTANCE_OHM                   						0x272A
#define BT_UUID_UNIT_ELECTRIC_CONDUCTANCE_SIEMENS              						0x272B
#define BT_UUID_UNIT_MAGNETIC_FLUX_WEBER                       						0x272C
#define BT_UUID_UNIT_MAGNETIC_FLUX_DENSITY_TESLA               						0x272D
#define BT_UUID_UNIT_INDUCTANCE_HENRY                          						0x272E
#define BT_UUID_UNIT_CELSIUS_TEMPERATURE                       						0x272F
#define BT_UUID_UNIT_LUMINOUS_FLUX_LUMEN                       						0x2730
#define BT_UUID_UNIT_ILLUMINANCE_LUX                           						0x2731
#define BT_UUID_UNIT_ACTIVITY_RADIONUCLIDE_BECQUEREL           						0x2732
#define BT_UUID_UNIT_ABSORBED_DOSE_GRAY                        						0x2733
#define BT_UUID_UNIT_DOSE_EQUIVALENT_SIEVERT                   						0x2734
#define BT_UUID_UNIT_CATALYTIC_ACTIVITY_KATAL                  						0x2735
#define BT_UUID_UNIT_DYNAMIC_VISCOSITY_PASCAL_SECOND           						0x2740
#define BT_UUID_UNIT_MOMENT_OF_FORCE_NEWTON_METRE              						0x2741
#define BT_UUID_UNIT_SURFACE_TENSION_NEWTON_PER_METRE          						0x2742
#define BT_UUID_UNIT_ANGULAR_VELOCITY_RADIAN_PER_SECOND        						0x2743
#define BT_UUID_UNIT_ANGULAR_ACCELERATION_RADIAN_PER_SECOND_SUQARED    				0x2744
#define BT_UUID_UNIT_HEAT_FLUX_DENSITY_WATT_PER_SQUARE_MEtRE   						0x2745
#define BT_UUID_UNIT_HEAT_CAPACITY_JOULE_PER_KELVIN            						0x2746
#define BT_UUID_UNIT_SPECIFIC_HEAT_CAPACITY_JOULE_PER_KG_KELVIN    					0x2747
#define BT_UUID_UNIT_SPECIFIC_ENERGY_JOULE_PEr_KG              						0x2748
#define BT_UUID_UNIT_THERMAL_CONDUCTIVITY_WATT_PER_METRE_KELVIN    					0x2749
#define BT_UUID_UNIT_ENERGY_DENSITY_JOULE_PER_CUBIC_METRE      						0x274A
#define BT_UUID_UNIT_ELECTRIC_FIELD_STRENGTH_VOLT_PER_METRE    						0x274B
#define BT_UUID_UNIT_ELECTRIC_CHARGE_DENSITY_COULOMB_PER_CUBIC_METRE				0x274C
#define BT_UUID_UNIT_SURFACE_CHARGE_DENSITY_COULOMB_PER_SQUARE_METRE				0x274D
#define BT_UUID_UNIT_ELECTRIC_FLUX_DENSITY_COULOMB_PER_SQUARE_METRE					0x274E
#define BT_UUID_UNIT_PERMITTIVITY_FARAD_PER_METRE              						0x274F
#define BT_UUID_UNIT_PERMEABILITY_HENRY_PER_METRE              						0x2750
#define BT_UUID_UNIT_MOLAR_ENERGY_JOULE_PER_MOLE               						0x2751
#define BT_UUID_UNIT_MOLAR_ENTROPY_JOULE_PER_MOLE_KELVIN       						0x2752
#define BT_UUID_UNIT_EXPOSURE_COULOMB_PER_KG                   						0x2753
#define BT_UUID_UNIT_ABSORBED_DOSE_RATE_GRAY_PER_SECOND        						0x2754
#define BT_UUID_UNIT_RADIAN_DENSITY_WATT_PER_STERADIAN         						0x2755
#define BT_UUID_UNIT_RADIANCE_WATT_PER_SQUARE_METRE_RADIAN     						0x2756
#define BT_UUID_UNIT_CATALYTIC_ACTIVITY_CONCENTRATION_KATAL_PER_CUBIC_METRE    		0x2757
#define BT_UUID_UNIT_TIME_MINUTE                               						0x2760
#define BT_UUID_UNIT_TIME_HOUR                                 						0x2761
#define BT_UUID_UNIT_TIME_DAY                                  						0x2762
#define BT_UUID_UNIT_PLANE_ANGLE_DEGREE                        						0x2763
#define BT_UUID_UNIT_PLANE_ANGLE_MINUTE                        						0x2764
#define BT_UUID_UNIT_PLANE_ANGLE_SECOND                        						0x2765
#define BT_UUID_UNIT_AREA_HECTARE                              						0x2766
#define BT_UUID_UNIT_VOLUME_LITRE                              						0x2767
#define BT_UUID_UNIT_MASS_TONNE                                						0x2768
#define BT_UUID_UNIT_PRESSURE_BAR                              						0x2780
#define BT_UUID_UNIT_PRESSURE_MILLIMETRE_OF_MERCURY            						0x2781
#define BT_UUID_UNIT_LENGTH_ANGSTROM                           						0x2782
#define BT_UUID_UNIT_LENGTH_NAUTICAL_MILE                      						0x2783
#define BT_UUID_UNIT_AREA_BARN                                 						0x2784
#define BT_UUID_UNIT_VELOCITY_KNOT                             						0x2785
#define BT_UUID_UNIT_LOGARITHMIC_RADIO_QUANTITY_NEPER          						0x2786
#define BT_UUID_UNIT_LOGARITHMIC_RADIO_QUANTITY_BEL            						0x2787
#define BT_UUID_UNIT_LENGTH_YARD                               						0x27A0
#define BT_UUID_UNIT_LENGTH_PARSEC                             						0x27A1
#define BT_UUID_UNIT_LENGTH_INCH                               						0x27A2
#define BT_UUID_UNIT_LENGTH_FOOT                               						0x27A3
#define BT_UUID_UNIT_LENGTH_MILE                               						0x27A4
#define BT_UUID_UNIT_PRESSURE_POUND_FORCE_PER_SQUARE_INCH      						0x27A5
#define BT_UUID_UNIT_VELOCITY_KILOMETRE_PER_HOUR               						0x27A6
#define BT_UUID_UNIT_VELOCITY_MILE_PER_HOUR                    						0x27A7
#define BT_UUID_UNIT_ANGULAR_VELOCITY_REVOLUTION_PER_MINUTE    						0x27A8
#define BT_UUID_UNIT_ENERGY_GRAM_CALORIE                       						0x27A9
#define BT_UUID_UNIT_ENERGY_KG_CALORIE                         						0x27AA
#define BT_UUID_UNIT_ENERGY_KWATT_HOUR                         						0x27AB
#define BT_UUID_UNIT_THERMODYNAMIC_TEMPERATURE_DEGREE_FAHRENHEIT   					0x27AC
#define BT_UUID_UNIT_PERCENTAGE                                						0x27AD
#define BT_UUID_UNIT_PER_MILLE                                 						0x27AE
#define BT_UUID_UNIT_PERIOD_BEATS_PER_MINUTE                   						0x27AF
#define BT_UUID_UNIT_ELECTRIC_CHARGE_AMPERE_HOURS              						0x27B0
#define BT_UUID_UNIT_MASS_DENSITY_MILLIGRAM_PER_DECILITRE      						0x27B1
#define BT_UUID_UNIT_MASS_DENSITY_MILLIMOLE_PER_LITRE          						0x27B2
#define BT_UUID_UNIT_TIME_YEAR                                 						0x27B3
#define BT_UUID_UNIT_TIME_MONTH                                						0x27B4
#define BT_UUID_UNIT_CONCENTRATION_COUNT_PER_CUBIC_METRE       						0x27B5
#define BT_UUID_UNIT_IRRADIANCE_WATT_PER_SQUARE_METRE          						0x27B6
#define BT_UUID_UNIT_MILLILITER_PER_KG_PER_MINUTE              						0x27B7
#define BT_UUID_UNIT_MASS_POUND                                						0x27B8
#define BT_UUID_UNIT_METABOLIC_EQUIVALENT                      						0x27B9
#define BT_UUID_UNIT_STEP_PER_MINUTE                           						0x27BA
#define BT_UUID_UNIT_STROKE_PER_MINUTE                         						0x27BC
#define BT_UUID_UNIT_PACE_KILOMETRE_PER_MINUTE                 						0x27BD
#define BT_UUID_UNIT_LUMINOUS_EFFICACY_LUMEN_PER_WATT          						0x27BE
#define BT_UUID_UNIT_LUMINOUS_ENERGY_LUMEN_HOUR                						0x27BF
#define BT_UUID_UNIT_LUMINOUS_EXPOSURE_LUX_HOUR                						0x27C0
#define BT_UUID_UNIT_MASS_FLOW_GRAM_PER_SECOND                 						0x27C1
#define BT_UUID_UNIT_VOLUME_FLOW_LITRE_PER_SECOND              						0x27C2
#define BT_UUID_UNIT_SOUND_PRESSURE_DECIBEL                    						0x27C3
#define BT_UUID_UNIT_PARTS_PER_MILLION                         						0x27C4
#define BT_UUID_UNIT_PARTS_PER_BILLION                         						0x27C5
#define BT_UUID_UNIT_MASS_DENSITY_REATE												0x27C6	// Mass density rate (mgram per decilitr per min)
#define BT_UUID_UNIT_ELECTRICAL_APPARENT_ENERGY										0x27C7	// Electrical Apparent Energy (KVolt amp hour)
#define BT_UUID_UNIT_ELECTRICAL_APPARENT_POWER										0x27C8	// Electrical Apparent Power (volt amp)

// 3.6 Declarations
#define BT_UUID_DECLARATIONS_PRIMARY_SERVICE                   						0x2800
#define BT_UUID_DECLARATIONS_SECONDARY_SERVICE                 						0x2801
#define BT_UUID_DECLARATIONS_INCLUDE                           						0x2802
#define BT_UUID_DECLARATIONS_CHARACTERISTIC                    						0x2803

// 3.7 Descriptor
#define BT_UUID_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES  						0x2900
#define BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION     						0x2901
#define BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION 						0x2902
#define BT_UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION 						0x2903
#define BT_UUID_DESCRIPTOR_CHARACTERISTIC_PRESENTATION_FORMAT  						0x2904
#define BT_UUID_DESCRIPTOR_CHARACTERISTIC_AGGREGATE_FORMAT     						0x2905
#define BT_UUID_DESCRIPTOR_VALID_RANGE                         						0x2906
#define BT_UUID_DESCRIPTOR_EXTERNAL_REPORT_REFERENCE           						0x2907
#define BT_UUID_DESCRIPTOR_REPORT_REFERENCE                    						0x2908
#define BT_UUID_DESCRIPTOR_NUMBER_OF_DIGITALS                  						0x2909
#define BT_UUID_DESCRIPTOR_VALUE_TRIGGER_SETTING               						0x290A
#define BT_UUID_DESCRIPTOR_ENVIRONMENTAL_SENSING_CONFIGURATION 						0x290B
#define BT_UUID_DESCRIPTOR_ENVIRONMENTAL_SENSING_MEASUREMENT   						0x290C
#define BT_UUID_DESCRIPTOR_ENVIRONMENTAL_SENSING_TRIGGER_SETTING   					0x290D
#define BT_UUID_DESCRIPTOR_TIME_TRIGGER_SETTING                						0x290E
#define BT_UUID_DESCRIPTOR_COMPLETE_BR_EDR_TRANSPORT_BLOCK_DATA    					0x290F
#define BT_UUID_DESCRIPTOR_OBSERVATION_SCHEDULE										0x2910
#define BT_UUID_DESCRIPTOR_VALID_RANGE_ACCURACY										0x2911

// 3.8 Characteristics
#define BT_UUID_CHARACTERISTIC_DEVICE_NAME                                    		0x2A00
#define BT_UUID_CHARACTERISTIC_APPEARANCE                                     		0x2A01
#define BT_UUID_CHARACTERISTIC_PERIPHERAL_PRIVACY_FLAG                        		0x2A02
#define BT_UUID_CHARACTERISTIC_RECONNECTION_ADDRESS                           		0x2A03
#define BT_UUID_CHARACTERISTIC_PERIPH_PREFERRED_CONN_PARAM							0x2A04
#define BT_UUID_CHARACTERISTIC_SERVICE_CHANGED										0x2A05
#define BT_UUID_CHARACTERISTIC_ALERT_LEVEL                                    		0x2A06
#define BT_UUID_CHARACTERISTIC_TX_POWER_LEVEL                                 		0x2A07
#define BT_UUID_CHARACTERISTIC_DATE_TIME                                      		0x2A08
#define BT_UUID_CHARACTERISTIC_DAY_OF_WEEK                                    		0x2A09
#define BT_UUID_CHARACTERISTIC_DAT_DATE_TIME                                  		0x2A0A
#define BT_UUID_CHARACTERISTIC_EXACT_TIME_256                                 		0x2A0C
#define BT_UUID_CHARACTERISTIC_DST_OFFSET                                     		0x2A0D
#define BT_UUID_CHARACTERISTIC_TIME_ZONE                                      		0x2A0E
#define BT_UUID_CHARACTERISTIC_LOCAL_TIME_INFO                                		0x2A0F
#define BT_UUID_CHARACTERISTIC_TIME_WITH_DST                                  		0x2A11
#define BT_UUID_CHARACTERISTIC_TIME_ACCURACY                                  		0x2A12
#define BT_UUID_CHARACTERISTIC_TIME_SOURCE                                    		0x2A13
#define BT_UUID_CHARACTERISTIC_REFERENCE_TIME_INFO                            		0x2A14
#define BT_UUID_CHARACTERISTIC_TIME_UPDATE_CONTROL_POINT                      		0x2A16
#define BT_UUID_CHARACTERISTIC_TIME_UPDATE_STATE                              		0x2A17
#define BT_UUID_CHARACTERISTIC_GLUCOSE_MEASUREMENT                            		0x2A18
#define BT_UUID_CHARACTERISTIC_BATTERY_LEVEL                                  		0x2A19
#define BT_UUID_CHARACTERISTIC_TEMPERATURE_MEASUREMENT                        		0x2A1C
#define BT_UUID_CHARACTERISTIC_TEMPERATURE_TYPE                               		0x2A1D
#define BT_UUID_CHARACTERISTIC_INTERMEDIATE_TEMPERATURE                       		0x2A1E
#define BT_UUID_CHARACTERISTIC_MEASUREMENT_INTERVAL                           		0x2A21
#define BT_UUID_CHARACTERISTIC_BOOT_KEYBOARD_INPUT_REPORT                     		0x2A22
#define BT_UUID_CHARACTERISTIC_SYSTEM_ID                                      		0x2A23
#define BT_UUID_CHARACTERISTIC_MODEL_NUMBER_STRING                            		0x2A24
#define BT_UUID_CHARACTERISTIC_SERIAL_NUMBER_STRING                           		0x2A25
#define BT_UUID_CHARACTERISTIC_FIRMWARE_REVISION_STRING                       		0x2A26
#define BT_UUID_CHARACTERISTIC_HARDWARE_REVISION_STRING                       		0x2A27
#define BT_UUID_CHARACTERISTIC_SOFTWARE_REVISION_STRING                       		0x2A28
#define BT_UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING                       		0x2A29
#define BT_UUID_CHARACTERISTIC_IEEE_11073_20601_REGULATORY_CERT_DATA_LIST     		0x2A2A
#define BT_UUID_CHARACTERISTIC_CURRENT_TIME                                   		0x2A2B
#define BT_UUID_CHARACTERISTIC_MAGNETIC_DECLINATION									0x2A2C
#define BT_UUID_CHARACTERISTIC_SCAN_REFRESH                                   		0x2A31
#define BT_UUID_CHARACTERISTIC_BOOT_KEYBOARD_OUTPUT_REPORT                    		0x2A32
#define BT_UUID_CHARACTERISTIC_BOOT_MOUSE_INPUT_REPORT                        		0x2A33
#define BT_UUID_CHARACTERISTIC_GLUCOSE_MEASUREMENT_CONTEXT                    		0x2A34
#define BT_UUID_CHARACTERISTIC_BLOOD_PRESSURE_MEASUREMENT                     		0x2A35
#define BT_UUID_CHARACTERISTIC_INTERMEDIATE_COFF_PRESSURE                     		0x2A36
#define BT_UUID_CHARACTERISTIC_HEART_RATE_MEASUREMENT                         		0x2A37
#define BT_UUID_CHARACTERISTIC_BODY_SENSOR_LOCATION                           		0x2A38
#define BT_UUID_CHARACTERISTIC_HEART_RATE_CONTROL_POINT                       		0x2A39
#define BT_UUID_CHARACTERISTIC_ALERT_STATUS                                   		0x2A3F
#define BT_UUID_CHARACTERISTIC_RINGER_CONTROL_POINT                           		0x2A40
#define BT_UUID_CHARACTERISTIC_RINGER_SETTING                                 		0x2A41
#define BT_UUID_CHARACTERISTIC_ALERT_CATEGORY_ID_BITMASK                      		0x2A42
#define BT_UUID_CHARACTERISTIC_ALERT_CATEGORY_ID                              		0x2A43
#define BT_UUID_CHARACTERISTIC_ALERT_NOTIFICATION_CONTROL_POINT               		0x2A44
#define BT_UUID_CHARACTERISTIC_UNREAD_ALERT_STATUS                            		0x2A45
#define BT_UUID_CHARACTERISTIC_NEW_ALERT                                      		0x2A46
#define BT_UUID_CHARACTERISTIC_SUPPORTED_NEW_ALERT_CATEGORY                   		0x2A47
#define BT_UUID_CHARACTERISTIC_SUPPORTED_UNREAD_ALERT_CATEGORY                		0x2A48
#define BT_UUID_CHARACTERISTIC_BLOOD_PRESSURE_FEATURE                         		0x2A49
#define BT_UUID_CHARACTERISTIC_HID_INFO                                       		0x2A4A
#define BT_UUID_CHARACTERISTIC_REPORT_MAP                                     		0x2A4B
#define BT_UUID_CHARACTERISTIC_HID_CONTROL_POINT                              		0x2A4C
#define BT_UUID_CHARACTERISTIC_REPORT                                         		0x2A4D
#define BT_UUID_CHARACTERISTIC_PROTOCOL_MODE                                  		0x2A4E
#define BT_UUID_CHARACTERISTIC_SCAN_INTERVAL_WINDOW                           		0x2A4F
#define BT_UUID_CHARACTERISTIC_PNP_ID                                         		0x2A50
#define BT_UUID_CHARACTERISTIC_GLUCOSE_FEATURE                                		0x2A51
#define BT_UUID_CHARACTERISTIC_RECORD_ACCESS_CONTROL_POINT                    		0x2A52
#define BT_UUID_CHARACTERISTIC_RSC_MEASUREMENT                                		0x2A53
#define BT_UUID_CHARACTERISTIC_RSC_FEATURE                                    		0x2A54
#define BT_UUID_CHARACTERISTIC_SC_CONTROL_POINT                               		0x2A55
#define BT_UUID_CHARACTERISTIC_AGGREGATE                                      		0x2A5A
#define BT_UUID_CHARACTERISTIC_CSC_MEASUREMENT                                		0x2A5B
#define BT_UUID_CHARACTERISTIC_CSC_FEATURE                                    		0x2A5C
#define BT_UUID_CHARACTERISTIC_SENSOR_LOCATION                                		0x2A5D
#define BT_UUID_CHARACTERISTIC_PLX_SPOT_CHECK_MEASUREMENT                     		0x2A5E
#define BT_UUID_CHARACTERISTIC_PLX_CONTINUOUS_MEASUREMENT                     		0x2A5F
#define BT_UUID_CHARACTERISTIC_PLX_FEATURES                                   		0x2A60
#define BT_UUID_CHARACTERISTIC_CYCLING_POWER_MEASUREMENT                      		0x2A63
#define BT_UUID_CHARACTERISTIC_CYCLING_POWER_VECTOR                           		0x2A64
#define BT_UUID_CHARACTERISTIC_CYCLING_POWER_FEATURE                          		0x2A65
#define BT_UUID_CHARACTERISTIC_CYCLING_POWER_CONTROL_POINT                    		0x2A66
#define BT_UUID_CHARACTERISTIC_LOCATION_AND_SPEED                             		0x2A67
#define BT_UUID_CHARACTERISTIC_NAVIGATION                                     		0x2A68
#define BT_UUID_CHARACTERISTIC_POSITION_QUALITY                               		0x2A69
#define BT_UUID_CHARACTERISTIC_LN_FEATURE                                     		0x2A6A
#define BT_UUID_CHARACTERISTIC_LN_CONTROL_POINT                               		0x2A6B
#define BT_UUID_CHARACTERISTIC_ELEVATION                                      		0x2A6C
#define BT_UUID_CHARACTERISTIC_PRESSURE                                       		0x2A6D
#define BT_UUID_CHARACTERISTIC_TEMPERATURE                                    		0x2A6E
#define BT_UUID_CHARACTERISTIC_HUMIDITY                                       		0x2A6F
#define BT_UUID_CHARACTERISTIC_TRUE_WIND_SPEED                                		0x2A70
#define BT_UUID_CHARACTERISTIC_TRUE_WIND_DIRECTION                            		0x2A71
#define BT_UUID_CHARACTERISTIC_APPARENT_WIND_SPEED                            		0x2A72
#define BT_UUID_CHARACTERISTIC_APPARENT_WIND_DIRECTION                        		0x2A73
#define BT_UUID_CHARACTERISTIC_GUST_FACTOR                                    		0x2A74
#define BT_UUID_CHARACTERISTIC_POLLEN_CONCENTRATION                           		0x2A75
#define BT_UUID_CHARACTERISTIC_UV_INDEX                                       		0x2A76
#define BT_UUID_CHARACTERISTIC_IRRADIANCE                                     		0x2A77
#define BT_UUID_CHARACTERISTIC_RAINFALL                                       		0x2A78
#define BT_UUID_CHARACTERISTIC_WIND_CHILL                                     		0x2A79
#define BT_UUID_CHARACTERISTIC_HEAT_INDEX                                     		0x2A7A
#define BT_UUID_CHARACTERISTIC_DEW_POINT                                      		0x2A7B
#define BT_UUID_CHARACTERISTIC_DESCRIPTOR_VALUE_CHANGED                       		0x2A7D
#define BT_UUID_CHARACTERISTIC_AEROBIC_HEART_RATE_LOWER_LIMIT                 		0x2A7E
#define BT_UUID_CHARACTERISTIC_AEROBIC_THRESHOLD                              		0x2A7F
#define BT_UUID_CHARACTERISTIC_AGE                                            		0x2A80
#define BT_UUID_CHARACTERISTIC_ANAEROBIC_HEART_RATE_LOWER_LIMIT               		0x2A81
#define BT_UUID_CHARACTERISTIC_ANAEROBIC_HEART_RATE_UPPER_LIMIT               		0x2A82
#define BT_UUID_CHARACTERISTIC_ANAEROBIC_THRESHOLD                            		0x2A83
#define BT_UUID_CHARACTERISTIC_AEROBIC_UPPER_LIMIT                            		0x2A84
#define BT_UUID_CHARACTERISTIC_DATE_OF_BIRTH                                  		0x2A85
#define BT_UUID_CHARACTERISTIC_DATE_OF_THRESHOLD_ASSESSMENT                   		0x2A86
#define BT_UUID_CHARACTERISTIC_EMAIL_ADDRESS                                  		0x2A87
#define BT_UUID_CHARACTERISTIC_FAT_BURN_HEART_RATE_LOWER_LIMIT                		0x2A88
#define BT_UUID_CHARACTERISTIC_FAT_BURN_HEART_RATE_UPPER_LIMIT                		0x2A89
#define BT_UUID_CHARACTERISTIC_FIRST_NAME                                     		0x2A8A
#define BT_UUID_CHARACTERISTIC_FIVE_ZONE_HEART_RATE_LIMITS                    		0x2A8B
#define BT_UUID_CHARACTERISTIC_GENDER                                         		0x2A8C
#define BT_UUID_CHARACTERISTIC_HEART_RATE_MAX                                 		0x2A8D
#define BT_UUID_CHARACTERISTIC_HEIGHT                                         		0x2A8E
#define BT_UUID_CHARACTERISTIC_HIP_CIRCUMFERENCE                              		0x2A8F
#define BT_UUID_CHARACTERISTIC_LAST_NAME                                      		0x2A90
#define BT_UUID_CHARACTERISTIC_MAXIMUM_RECOMMENDED_HEART_RATE                 		0x2A91
#define BT_UUID_CHARACTERISTIC_RESTING_HEART_RATE                             		0x2A92
#define BT_UUID_CHARACTERISTIC_SPORT_TYPE_AEROBIC_ANAEROBIC_THRESHOLD         		0x2A93
#define BT_UUID_CHARACTERISTIC_THREE_ZONE_HEART_RATE_LIMITS                   		0x2A94
#define BT_UUID_CHARACTERISTIC_TWO_ZONE_HEART_RATE_LIMITS                     		0x2A95
#define BT_UUID_CHARACTERISTIC_VO2_MAX                                        		0x2A96
#define BT_UUID_CHARACTERISTIC_WAIST_CIRCUMFERENCE                            		0x2A97
#define BT_UUID_CHARACTERISTIC_WEIGHT                                         		0x2A98
#define BT_UUID_CHARACTERISTIC_DATABASE_CHANGE_INCREMENT                      		0x2A99
#define BT_UUID_CHARACTERISTIC_USER_INDEX                                     		0x2A9A
#define BT_UUID_CHARACTERISTIC_BODY_COMPOSITION_FEATURE                       		0x2A9B
#define BT_UUID_CHARACTERISTIC_BODY_COMPOSITION_MEASUREMENT                   		0x2A9C
#define BT_UUID_CHARACTERISTIC_WEIGHT_MEASUREMENT                             		0x2A9D
#define BT_UUID_CHARACTERISTIC_WEIGHT_SCALE_FEATURE                           		0x2A9E
#define BT_UUID_CHARACTERISTIC_USER_CONTROL_POINT                             		0x2A9F
#define BT_UUID_CHARACTERISTIC_MAGNETIC_FLUX_DENSITY_2D                       		0x2AA0
#define BT_UUID_CHARACTERISTIC_MAGNETIC_FLUX_DENSITY_3D                       		0x2AA1
#define BT_UUID_CHARACTERISTIC_LANGUAGE                                       		0x2AA2
#define BT_UUID_CHARACTERISTIC_BAROMETRIC_PRESSURE_TREND                      		0x2AA3
#define BT_UUID_CHARACTERISTIC_BOND_MANAGEMENT_CONTROL_POINT                  		0x2AA4
#define BT_UUID_CHARACTERISTIC_BOND_MANAGEMENT_FEATURE                        		0x2AA5
#define BT_UUID_CHARACTERISTIC_CENTRAL_ADDRESS_RESOLUTION                     		0x2AA6
#define BT_UUID_CHARACTERISTIC_CGM_MEASUREMENT                                		0x2AA7
#define BT_UUID_CHARACTERISTIC_CGM_FEATURE                                    		0x2AA8
#define BT_UUID_CHARACTERISTIC_CGM_STATUS                                     		0x2AA9
#define BT_UUID_CHARACTERISTIC_CGM_SESSION_START_TIME                         		0x2AAA
#define BT_UUID_CHARACTERISTIC_CGM_SESSION_RUN_TIME                           		0x2AAB
#define BT_UUID_CHARACTERISTIC_CGM_SPECIFIC_OPS_CONTROL_POINT                 		0x2AAC
#define BT_UUID_CHARACTERISTIC_INDOOR_POSITIONING_CONFIGURATION               		0x2AAD
#define BT_UUID_CHARACTERISTIC_LATITUDE                                       		0x2AAE
#define BT_UUID_CHARACTERISTIC_LONGITUDE                                      		0x2AAF
#define BT_UUID_CHARACTERISTIC_LOCAL_NORTH_COORDINATE                         		0x2AB0
#define BT_UUID_CHARACTERISTIC_LOCAL_EAST_COORDINATE                          		0x2AB1
#define BT_UUID_CHARACTERISTIC_FLOOR_NUMBER                                   		0x2AB2
#define BT_UUID_CHARACTERISTIC_ATTITUDE                                       		0x2AB3
#define BT_UUID_CHARACTERISTIC_UNCERTAINTY                                    		0x2AB4
#define BT_UUID_CHARACTERISTIC_LOCATION_NAME                                  		0x2AB5
#define BT_UUID_CHARACTERISTIC_URI                                            		0x2AB6
#define BT_UUID_CHARACTERISTIC_HTTP_HEADERS                                   		0x2AB7
#define BT_UUID_CHARACTERISTIC_HTTP_STATUS_CODE                               		0x2AB8
#define BT_UUID_CHARACTERISTIC_HTTP_ENTITY_BODY                               		0x2AB9
#define BT_UUID_CHARACTERISTIC_HTTP_CONTROL_POINT                             		0x2ABA
#define BT_UUID_CHARACTERISTIC_HTTPS_SECURITY                                 		0x2ABB
#define BT_UUID_CHARACTERISTIC_TDS_CONTROL_POINT                              		0x2ABC
#define BT_UUID_CHARACTERISTIC_OTS_FEATURE                                    		0x2ABD
#define BT_UUID_CHARACTERISTIC_OBJECT_NAME                                    		0x2ABE
#define BT_UUID_CHARACTERISTIC_OBJECT_TYPE                                    		0x2ABF
#define BT_UUID_CHARACTERISTIC_OBJECT_SIZE                                    		0x2AC0
#define BT_UUID_CHARACTERISTIC_OBJECT_FIRST_CREATED                           		0x2AC1
#define BT_UUID_CHARACTERISTIC_OBJECT_LAST_CREATED                            		0x2AC2
#define BT_UUID_CHARACTERISTIC_OBJECT_ID                                      		0x2AC3
#define BT_UUID_CHARACTERISTIC_OBJECT_PROPERTIES                              		0x2AC4
#define BT_UUID_CHARACTERISTIC_OBJECT_ACTION_CONTROL_POINT                    		0x2AC5
#define BT_UUID_CHARACTERISTIC_OBJECT_LIST_CONTROL_POINT                      		0x2AC6
#define BT_UUID_CHARACTERISTIC_OBJECT_LIST_FILTER                             		0x2AC7
#define BT_UUID_CHARACTERISTIC_OBJECT_CHANGED                                 		0x2AC8
#define BT_UUID_CHARACTERISTIC_RESOLVABLE_PRIVATE_ADDRESS_ONLY                		0x2AC9
//#define BT_UUID_CHARACTERISTIC_UNSPECIFIED                                    		0x2ACA
//#define BT_UUID_CHARACTERISTIC_DIRECTORY_LISTING                              		0x2ACB
#define BT_UUID_CHARACTERISTIC_FITNESS_MACHINE_FEATURE                        		0x2ACC
#define BT_UUID_CHARACTERISTIC_TREADMILL_DATA                                 		0x2ACD
#define BT_UUID_CHARACTERISTIC_CROSS_TRAINER_DATA                             		0x2ACE
#define BT_UUID_CHARACTERISTIC_STEP_CLIMBER_DATA                              		0x2ACF
#define BT_UUID_CHARACTERISTIC_STAIR_CLIMBER_DATA                             		0x2AD0
#define BT_UUID_CHARACTERISTIC_ROWER_DATA                                     		0x2AD1
#define BT_UUID_CHARACTERISTIC_INDOOR_BIKE_DATA                               		0x2AD2
#define BT_UUID_CHARACTERISTIC_TRAINING_STATUS                                		0x2AD3
#define BT_UUID_CHARACTERISTIC_SUPPORTED_SPEED_RANGE                          		0x2AD4
#define BT_UUID_CHARACTERISTIC_SUPPORTED_INCLINATION_RANGE                    		0x2AD5
#define BT_UUID_CHARACTERISTIC_SUPPORTED_RESISTANCE_LEVEL_RANGE               		0x2AD6
#define BT_UUID_CHARACTERISTIC_SUPPORTED_HEART_RATE_RANGE                     		0x2AD7
#define BT_UUID_CHARACTERISTIC_SUPPORTED_POWER_RANGE                          		0x2AD8
#define BT_UUID_CHARACTERISTIC_FITNESS_MACHINE_CONTROL_POINT                  		0x2AD9
#define BT_UUID_CHARACTERISTIC_FITNESS_MACHINE_STATUS                         		0x2ADA
#define BT_UUID_CHARACTERISTIC_MESH_PROVISIONING_DATA_IN                      		0x2ADB
#define BT_UUID_CHARACTERISTIC_MESH_PROVISIONING_DATA_OUT                     		0x2ADC
#define BT_UUID_CHARACTERISTIC_MESH_PROXY_DATA_IN                             		0x2ADD
#define BT_UUID_CHARACTERISTIC_MESH_PROXY_DATA_OUT                            		0x2ADE
#define BT_UUID_CHARACTERISTIC_AVERAGE_CURRENT                                		0x2AE0
#define BT_UUID_CHARACTERISTIC_AVERAGE_VOLTAGE                                		0x2AE1
#define BT_UUID_CHARACTERISTIC_BOOLEAN                                        		0x2AE2
#define BT_UUID_CHARACTERISTIC_CHROMATIC_DISTANCE_FROM_PLANCKIAN              		0x2AE3
#define BT_UUID_CHARACTERISTIC_CHROMATICITY_COORDINATES                       		0x2AE4
#define BT_UUID_CHARACTERISTIC_CHROMATICITY_IN_CCT_AND_DUV_VALUES             		0x2AE5
#define BT_UUID_CHARACTERISTIC_CHROMATICITY_TOLERANCE                         		0x2AE6
#define BT_UUID_CHARACTERISTIC_CIE_13_3_1995_COLOR_RENDERING_INDEX            		0x2AE7
#define BT_UUID_CHARACTERISTIC_COEFFICIENT                                    		0x2AE8
#define BT_UUID_CHARACTERISTIC_CORRELATED_COLOR_TEMPERATURE                   		0x2AE9
#define BT_UUID_CHARACTERISTIC_COUNT16                                        		0x2AEA
#define BT_UUID_CHARACTERISTIC_COUNT24                                        		0x2AEB
#define BT_UUID_CHARACTERISTIC_COUNTRY_CODE                                   		0x2AEC
#define BT_UUID_CHARACTERISTIC_DATE_UTC                                       		0x2AED
#define BT_UUID_CHARACTERISTIC_ELECTRIC_CURRENT                               		0x2AEE
#define BT_UUID_CHARACTERISTIC_ELECTRIC_CURRENT_RANGE                         		0x2AEF
#define BT_UUID_CHARACTERISTIC_ELECTRIC_CURRENT_SPECIFICATION                 		0x2AF0
#define BT_UUID_CHARACTERISTIC_ELECTRIC_CURRENT_STATISTICS                    		0x2AF1
#define BT_UUID_CHARACTERISTIC_ENERGY                                         		0x2AF2
#define BT_UUID_CHARACTERISTIC_ENERGY_IN_A_PERIOD_OF_DAY                      		0x2AF3
#define BT_UUID_CHARACTERISTIC_EVENT_STATISTICS                               		0x2AF4
#define BT_UUID_CHARACTERISTIC_FIXED_STRING16                                 		0x2AF5
#define BT_UUID_CHARACTERISTIC_FIXED_STRING24                                 		0x2AF6
#define BT_UUID_CHARACTERISTIC_FIXED_STRING36                                 		0x2AF7
#define BT_UUID_CHARACTERISTIC_FIXED_STRING8                                  		0x2AF8
#define BT_UUID_CHARACTERISTIC_GENERIC_LEVEL                                  		0x2AF9
#define BT_UUID_CHARACTERISTIC_GLOBAL_TRADE_ITEM_NUMBER                       		0x2AFA
#define BT_UUID_CHARACTERISTIC_ILLUMINACE                                     		0x2AFB
#define BT_UUID_CHARACTERISTIC_LUMINOUS_EFFICACY                              		0x2AFC
#define BT_UUID_CHARACTERISTIC_LUMINOUS_ENERGY                                		0x2AFD
#define BT_UUID_CHARACTERISTIC_LUMINOUS_EXPOSURE                              		0x2AFE
#define BT_UUID_CHARACTERISTIC_LUMINOUS_FLUX                                  		0x2AFF
#define BT_UUID_CHARACTERISTIC_LUMINOUS_FLUX_RANGE                            		0x2B00
#define BT_UUID_CHARACTERISTIC_LUMINOUS_INTENSITY                             		0x2B01
#define BT_UUID_CHARACTERISTIC_MASS_FLOW                                      		0x2B02
#define BT_UUID_CHARACTERISTIC_PERCEIVED_LIGHTNESS                            		0x2B03
#define BT_UUID_CHARACTERISTIC_PERCENTAGE8                                    		0x2B04
#define BT_UUID_CHARACTERISTIC_POWER                                          		0x2B05
#define BT_UUID_CHARACTERISTIC_POWER_SPECIFICATION                            		0x2B06
#define BT_UUID_CHARACTERISTIC_RELATIVE_RUNTIME_CURRENT_RANGE                 		0x2B07
#define BT_UUID_CHARACTERISTIC_RELATIVE_RUNTIME_GENERIC_LEVEL_RANGE           		0x2B08
#define BT_UUID_CHARACTERISTIC_RELATIVE_VALUE_VOLTAGE_RANGE                   		0x2B09
#define BT_UUID_CHARACTERISTIC_RELATIVE_VALUE_ILLUMINANCE_RANGE               		0x2B0A
#define BT_UUID_CHARACTERISTIC_RELATIVE_VALUE_PERIOD_OF_DAY                   		0x2B0B
#define BT_UUID_CHARACTERISTIC_RELATIVE_VALUE_TEMPERATURE_RANGE               		0x2B0C
#define BT_UUID_CHARACTERISTIC_TEMPERATURE8                                   		0x2B0D
#define BT_UUID_CHARACTERISTIC_TEMPERATURE8_PERIOD_OF_DAY                     		0x2B0E
#define BT_UUID_CHARACTERISTIC_TEMPERATURE8_STATISTICS                        		0x2B0F
#define BT_UUID_CHARACTERISTIC_TEMPERATURE_RANGE                              		0x2B10
#define BT_UUID_CHARACTERISTIC_TEMPERATURE_STATISTICS                         		0x2B11
#define BT_UUID_CHARACTERISTIC_TIME_DECIHOUR_8                                		0x2B12
#define BT_UUID_CHARACTERISTIC_TIMER_EXPONENTIAL_8                            		0x2B13
#define BT_UUID_CHARACTERISTIC_TIME_HOUR_24                                   		0x2B14
#define BT_UUID_CHARACTERISTIC_TIME_MILLISECOND_24                            		0x2B15
#define BT_UUID_CHARACTERISTIC_TIME_SECOND_16                                 		0x2B16
#define BT_UUID_CHARACTERISTIC_TIME_SECOND_8                                  		0x2B17
#define BT_UUID_CHARACTERISTIC_VOLTAGE                                        		0x2B18
#define BT_UUID_CHARACTERISTIC_VOLTAGE_SPECIFICATION                          		0x2B19
#define BT_UUID_CHARACTERISTIC_VOLTAGE_STATISTICS                             		0x2B1A
#define BT_UUID_CHARACTERISTIC_VOLUME_FLOW                                    		0x2B1B
#define BT_UUID_CHARACTERISTIC_CHROMATICITY_COORDINATES1                       		0x2B1C
#define BT_UUID_CHARACTERISTIC_RC_FEATURE                                     		0x2B1D
#define BT_UUID_CHARACTERISTIC_RC_SETTINGS                                    		0x2B1E
#define BT_UUID_CHARACTERISTIC_RECONNECTION_CONFIG_CONTROL_POINT              		0x2B1F
#define BT_UUID_CHARACTERISTIC_IDD_STATUS_CHANGED                             		0x2B20
#define BT_UUID_CHARACTERISTIC_IDD_STATUS                                     		0x2B21
#define BT_UUID_CHARACTERISTIC_IDD_ANNUNCIATION_STATUS                        		0x2B22
#define BT_UUID_CHARACTERISTIC_IDD_FEATURES                                   		0x2B23
#define BT_UUID_CHARACTERISTIC_IDD_STATUS_READER_CONTROL_POINT                		0x2B24
#define BT_UUID_CHARACTERISTIC_IDD_COMMAND_CONTROL_POINT                      		0x2B25
#define BT_UUID_CHARACTERISTIC_IDD_COMMAND_DATA                               		0x2B26
#define BT_UUID_CHARACTERISTIC_IDD_RECORD_ACCESS_CONTROL_POINT                		0x2B27
#define BT_UUID_CHARACTERISTIC_IDD_HISTORY_DATA                               		0x2B28
#define BT_UUID_CHARACTERISTIC_CLIENT_SUPPORT_FEATURES                        		0x2B29
#define BT_UUID_CHARACTERISTIC_DATABASE_HASH                                  		0x2B2A
#define BT_UUID_CHARACTERISTIC_BSS_CONTROL_POINT                              		0x2B2B
#define BT_UUID_CHARACTERISTIC_BSS_RESPONSE                                   		0x2B2C
#define BT_UUID_CHARACTERISTIC_EMERGENCY_ID                                   		0x2B2D
#define BT_UUID_CHARACTERISTIC_EMERGENCY_TEXT                                 		0x2B2E
#define BT_UUID_CHARACTERISTIC_ACS_STATUS											0x2B2F
#define BT_UUID_CHARACTERISTIC_ACS_DATA_IN											0x2B30
#define BT_UUID_CHARACTERISTIC_ACS_DATA_OUT_NOTIF									0x2B31
#define BT_UUID_CHARACTERISTIC_ACS_DATA_OUT_IND										0x2B32
#define BT_UUID_CHARACTERISTIC_ACS_CONTROL_POINT									0x2B33
#define BT_UUID_CHARACTERISTIC_ENHANCED_BLOOD_PRESSURE_MEASUREMENT            		0x2B34
#define BT_UUID_CHARACTERISTIC_ENHENCED_INTERMEDIATE_CUFF_PRESSURE            		0x2B35
#define BT_UUID_CHARACTERISTIC_BLOOD_PRESSURE_RECORD                          		0x2B36
#define BT_UUID_CHARACTERISTIC_REGISTERED_USER										0x2B37
#define BT_UUID_CHARACTERISTIC_BR_EDR_HANDOVER_DATA                           		0x2B38
#define BT_UUID_CHARACTERISTIC_BLUETOOTH_SIG_DATA                             		0x2B39
#define BT_UUID_CHARACTERISTIC_SERVER_SUPPORTED_FEATURES                      		0x2B3A
#define BT_UUID_CHARACTERISTIC_PHYSICAL_ACTIVITY_MONITOR_FEATURE              		0x2B3B
#define BT_UUID_CHARACTERISTIC_GENERAL_ACTIVITY_INSTANTANEOUS_DATA            		0x2B3C
#define BT_UUID_CHARACTERISTIC_GENERAL_ACTIVITY_SUMMARY_DATA                  		0x2B3D
#define BT_UUID_CHARACTERISTIC_CARDIO_RESPYRATORY_ACTIVITY_INSTANTANEOUS_DATA 		0x2B3E
#define BT_UUID_CHARACTERISTIC_CARDIO_RESPIRATORY_ACTIVITY_SUMMARY_DATA       		0x2B3F
#define BT_UUID_CHARACTERISTIC_STEP_COUNTER_ACTIVITY_SUMMARY_DATA             		0x2B40
#define BT_UUID_CHARACTERISTIC_SLEEP_ACTIVITY_INSTANTANEOUS_DATA              		0x2B41
#define BT_UUID_CHARACTERISTIC_SLEEP_ACTIVITY_SUMMARY_DATA                    		0x2B42
#define BT_UUID_CHARACTERISTIC_PHYSICAL_ACTIVITY_MONITOR_CONTROL_POINT        		0x2B43
#define BT_UUID_CHARACTERISTIC_CURRENT_SESSION                                		0x2B44
#define BT_UUID_CHARACTERISTIC_SESSION                                        		0x2B45
#define BT_UUID_CHARACTERISTIC_PREFERED_UNITS                                 		0x2B46
#define BT_UUID_CHARACTERISTIC_HIGH_RESOLUTION_HEIGHT                         		0x2B47
#define BT_UUID_CHARACTERISTIC_MIDDLE_NAME                                    		0x2B48
#define BT_UUID_CHARACTERISTIC_STRIDE_LENGTH                                  		0x2B49
#define BT_UUID_CHARACTERISTIC_HANDEDNESS                                     		0x2B4A
#define BT_UUID_CHARACTERISTIC_DEVICE_WEARING_POSITION                        		0x2B4B
#define BT_UUID_CHARACTERISTIC_FOUR_ZONE_HEART_RATE_LIMITS                    		0x2B4C
#define BT_UUID_CHARACTERISTIC_HIGH_INTENSITY_EXERCISE_THRESHOLD              		0x2B4D
#define BT_UUID_CHARACTERISTIC_ACTIVITY_GOAL                                  		0x2B4E
#define BT_UUID_CHARACTERISTIC_SEDENTARY_INTERVAL_NOTIFICATION                		0x2B4F
#define BT_UUID_CHARACTERISTIC_CALORIC_INTAKE                                 		0x2B50
#define BT_UUID_CHARACTERISTIC_TMAP_ROLE_CHARACTERISTIC                       		0x2B51
#define BT_UUID_CHARACTERISTIC_AUDIO_INPUT_STATE                              		0x2B77
#define BT_UUID_CHARACTERISTIC_GAIN_SETTING_ATTRIBUTE                         		0x2B78
#define BT_UUID_CHARACTERISTIC_AUDIO_INPUT_TYPE                               		0x2B79
#define BT_UUID_CHARACTERISTIC_AUDIO_INPUT_STATUS                             		0x2B7A
#define BT_UUID_CHARACTERISTIC_AUDIO_INPUT_CONTROL_POINT                      		0x2B7B
#define BT_UUID_CHARACTERISTIC_AUDIO_INPUT_DESCRIPTION                        		0x2B7C
#define BT_UUID_CHARACTERISTIC_VOLUME_STATE                                   		0x2B7D
#define BT_UUID_CHARACTERISTIC_VOLUME_CONTROL_POINT                           		0x2B7E
#define BT_UUID_CHARACTERISTIC_VOLUME_FLAGS                                   		0x2B7F
#define BT_UUID_CHARACTERISTIC_OFFSET_STATE                                   		0x2B80
#define BT_UUID_CHARACTERISTIC_AUDIO_LOCATION                                 		0x2B81
#define BT_UUID_CHARACTERISTIC_VOLUME_OFFSET_CONTROL_POINT                    		0x2B82
#define BT_UUID_CHARACTERISTIC_AUDIO_OUTPUT_DESCRIPTION                       		0x2B83
#define BT_UUID_CHARACTERISTIC_SET_IDENTITY_RESOLVING_KEY_CHARACTERISTIC      		0x2B84
#define BT_UUID_CHARACTERISTIC_SIZE_CHARACTERISTIC                            		0x2B85
#define BT_UUID_CHARACTERISTIC_LOCK_CHARACTERISTIC                            		0x2B86
#define BT_UUID_CHARACTERISTIC_RANK_CHARACTERISTIC                            		0x2B87
#define BT_UUID_CHARACTERISTIC_ENCRYPTED_DATA_KEY_MATERIAL							0x2B88
#define BT_UUID_CHARACTERISTIC_APPARENT_ENERGY										0x2B89
#define BT_UUID_CHARACTERISTIC_APPARENT_POWER										0x2B8A
#define BT_UUID_CHARACTERISTIC_LIFE_HEALTH_OBSERVATIONS								0x2B8B
#define BT_UUID_CHARACTERISTIC_CO2_CONCENTRATION									0x2B8C
#define BT_UUID_CHARACTERISTIC_COSINE_ANGLE											0x2B8D
#define BT_UUID_CHARACTERISTIC_DEVICE_TIME_FEATURE                            		0x2B8E
#define BT_UUID_CHARACTERISTIC_DEVICE_TIME_PARAMETERS                         		0x2B8F
#define BT_UUID_CHARACTERISTIC_DEVICE_TIME                                    		0x2B90
#define BT_UUID_CHARACTERISTIC_DEVICE_TIME_CONTROL_POINT                      		0x2B91
#define BT_UUID_CHARACTERISTIC_TIME_CHANGE_LOG_DATA                           		0x2B92
#define BT_UUID_CHARACTERISTIC_MEDIA_PLAYER_NAME                              		0x2B93
#define BT_UUID_CHARACTERISTIC_MEDIA_PLAYER_ICON_OBJECT_ID                    		0x2B94
#define BT_UUID_CHARACTERISTIC_MEDIA_PLAYER_ICON_URL                          		0x2B95
#define BT_UUID_CHARACTERISTIC_TRACK_CHANGED                                  		0x2B96
#define BT_UUID_CHARACTERISTIC_TRACK_TITLE                                    		0x2B97
#define BT_UUID_CHARACTERISTIC_TRACK_DURATION                                 		0x2B98
#define BT_UUID_CHARACTERISTIC_TRACK_POSITION                                 		0x2B99
#define BT_UUID_CHARACTERISTIC_PALYBACK_SPEED                                 		0x2B9A
#define BT_UUID_CHARACTERISTIC_SEEKING_SPEED                                  		0x2B9B
#define BT_UUID_CHARACTERISTIC_CURRENT_TRACK_SEGMENT_OBJECT_ID                		0x2B9C
#define BT_UUID_CHARACTERISTIC_CURRENT_TRACK_OBJECT_ID                        		0x2B9D
#define BT_UUID_CHARACTERISTIC_NEXT_TRACK_OBJECT_ID                           		0x2B9E
#define BT_UUID_CHARACTERISTIC_PARENT_GROUP_OBJECT_ID                         		0x2B9F
#define BT_UUID_CHARACTERISTIC_CURRENT_GROUP_OBJECT_ID                        		0x2BA0
#define BT_UUID_CHARACTERISTIC_PLAYING_ORDER                                  		0x2BA1
#define BT_UUID_CHARACTERISTIC_PLAYING_ORDER_SUPPORTED                        		0x2BA2
#define BT_UUID_CHARACTERISTIC_MEDIA_STATE                                    		0x2BA3
#define BT_UUID_CHARACTERISTIC_MEDIA_CONTROL_POINT                            		0x2BA4
#define BT_UUID_CHARACTERISTIC_MEDIA_CONTROL_POINT_OPCODES_SUPPORTED          		0x2BA5
#define BT_UUID_CHARACTERISTIC_SEARCH_RESULTS_OBJECT_ID                       		0x2BA6
#define BT_UUID_CHARACTERISTIC_SEARCH_CONTROL_POINT                           		0x2BA7
#define BT_UUID_CHARACTERISTIC_MEDIA_PLAYER_ICON_OBJECT_TYPE                  		0x2BA9
#define BT_UUID_CHARACTERISTIC_TRACK_SEGMENTS_OBJECT_TYPE                     		0x2BAA
#define BT_UUID_CHARACTERISTIC_TRACK_OBJECT_TYPE                              		0x2BAB
#define BT_UUID_CHARACTERISTIC_GROUP_OBJECT_TYPE                              		0x2BAC
#define BT_UUID_CHARACTERISTIC_CONSTANT_TONE_EXTENSION_ENABLE                 		0x2BAD
#define BT_UUID_CHARACTERISTIC_ADVERTISING_CONSTANT_TONE_EXTEN_MIN_LENGTH     		0x2BAE
#define BT_UUID_CHARACTERISTIC_ADVERTISING_CONSTANT_TONE_EXTEN_MIN_TX_COUNT   		0x2BAF
#define BT_UUID_CHARACTERISTIC_ADVERTISING_CONSTANT_TONE_EXTEN_TX_DURATION    		0x2BB0
#define BT_UUID_CHARACTERISTIC_ADVERTISING_CONSTANT_TONE_EXTEN_INTERVAL       		0x2BB1
#define BT_UUID_CHARACTERISTIC_ADVERTISING_CONSTANT_TONE_EXTEN_PHY            		0x2BB2
#define BT_UUID_CHARACTERISTIC_BEARER_PROVIDER_NAME                           		0x2BB3
#define BT_UUID_CHARACTERISTIC_BEARER_UCI                                     		0x2BB4
#define BT_UUID_CHARACTERISTIC_BEARER_TECHNOLOTY                              		0x2BB5
#define BT_UUID_CHARACTERISTIC_BEARER_URI_SCHEMES_SUPPORTED_LIST              		0x2BB6
#define BT_UUID_CHARACTERISTIC_BEARER_SIGNAL_STRENGTH                         		0x2BB7
#define BT_UUID_CHARACTERISTIC_BEARER_SIGNAL_STRENGTH_REPORTING_INTERVAL      		0x2BB8
#define BT_UUID_CHARACTERISTIC_BEARER_LIST_CURRENT_CALLS                      		0x2BB9
#define BT_UUID_CHARACTERISTIC_CONTENT_CONTROL_ID                             		0x2BBA
#define BT_UUID_CHARACTERISTIC_STATUS_FLAGS                                   		0x2BBB
#define BT_UUID_CHARACTERISTIC_INCOMING_CALL_TARGET_BEARER_URI                		0x2BBC
#define BT_UUID_CHARACTERISTIC_CALL_STATE                                     		0x2BBD
#define BT_UUID_CHARACTERISTIC_CALL_CONTROL_POINT                             		0x2BBE
#define BT_UUID_CHARACTERISTIC_CALL_CONTROL_POINT_OPTIONAL_OPCODES            		0x2BBF
#define BT_UUID_CHARACTERISTIC_TERMINATION_REASON                             		0x2BC0
#define BT_UUID_CHARACTERISTIC_INCOMING_CALL                                  		0x2BC1
#define BT_UUID_CHARACTERISTIC_CALL_FRIENDLY_NAME                             		0x2BC2
#define BT_UUID_CHARACTERISTIC_MUTE                                           		0x2BC3
#define BT_UUID_CHARACTERISTIC_SINK_ASE                                       		0x2BC4
#define BT_UUID_CHARACTERISTIC_SOURCE_ASE                                     		0x2BC5
#define BT_UUID_CHARACTERISTIC_ASE_CONTROL_POINT                              		0x2BC6
#define BT_UUID_CHARACTERISTIC_BROADCAST_AUDIO_SCAN_CONTROL_POINT             		0x2BC7
#define BT_UUID_CHARACTERISTIC_BROADCAST_RECEIVE_STATE                        		0x2BC8
#define BT_UUID_CHARACTERISTIC_SINK_PAC                                       		0x2BC9
#define BT_UUID_CHARACTERISTIC_SINK_AUDIO_LOCATIONS                           		0x2BCA
#define BT_UUID_CHARACTERISTIC_SOURCE_PAC                                     		0x2BCB
#define BT_UUID_CHARACTERISTIC_SOURCE_AUDIO_LOCATIONS                         		0x2BCC
#define BT_UUID_CHARACTERISTIC_AVAILABLE_AUDIO_CONTEXTS                       		0x2BCD
#define BT_UUID_CHARACTERISTIC_SUPPORTED_AUDIO_CONTEXTS                       		0x2BCE
#define BT_UUID_CHARACTERISTIC_AMMONIA_CONCENTRATION                          		0x2BCF
#define BT_UUID_CHARACTERISTIC_CARBON_MONOXIDE_CONCENTRATION                  		0x2BD0
#define BT_UUID_CHARACTERISTIC_METHANE_CONCENTRATION                          		0x2BD1
#define BT_UUID_CHARACTERISTIC_NITROGEN_DIOXIDE_CONCENTRATION                 		0x2BD2
#define BT_UUID_CHARACTERISTIC_NON_METHANE_VOLATILE_ORGANIC_COMPOUNDS_CONCENTRATION 0x2BD3
#define BT_UUID_CHARACTERISTIC_OZONE_COCENTRATION                             		0x2BD4
#define BT_UUID_CHARACTERISTIC_PARTICULATE_MATTER_PM1_CONCENTRATION           		0x2BD5  // Particulate Matter - PM1 concentration
#define BT_UUID_CHARACTERISTIC_PARTICULATE_MATTER_PM2_5_CONCENTRATION         		0x2BD6  // Particulate Matter - PM2.5 concentration
#define BT_UUID_CHARACTERISTIC_PARTICULATE_MATTER_PM10_CONCENTRATION          		0x2BD7  // Particulate Matter - PM10 concentration
#define BT_UUID_CHARACTERISTIC_SULFUR_DIOXIDE_CONCENTRATION                   		0x2BD8
#define BT_UUID_CHARACTERISTIC_SULFUR_HEXAFLUORIDE_CONCENTRATION              		0x2BD9
#define BT_UUID_CHARACTERISTIC_HEARING_AID_FEATURES                           		0x2BDA // could be changed
#define BT_UUID_CHARACTERISTIC_HEARING_AID_PRESET_CONTROL_POINT               		0x2BDB // could be changed
#define BT_UUID_CHARACTERISTIC_ACTIVE_PRESET_INDEX                            		0x2BDC // could be changed
#define BT_UUID_CHARACTERISTIC_STORED_HEALTH_OBSERVATIONS							0x2BDD
#define BT_UUID_CHARACTERISTIC_FIXED_STRING64										0x2BDE
#define BT_UUID_CHARACTERISTIC_HIGH_TEMPERATURE										0x2BDF
#define BT_UUID_CHARACTERISTIC_HIGH_VOLTAGE											0x2BE0
#define BT_UUID_CHARACTERISTIC_LIGHT_DISTRIBUTION									0x2BE1
#define BT_UUID_CHARACTERISTIC_LIGHT_OUTPUT											0x2BE2
#define BT_UUID_CHARACTERISTIC_LIGHT_SOURCE_TYPE									0x2BE3
#define BT_UUID_CHARACTERISTIC_NOISE												0x2BE4
#define BT_UUID_CHARACTERISTIC_REL_RUNTIME_CORR_COLOR_TEMP_RANGE					0x2BE5
#define BT_UUID_CHARACTERISTIC_TIME_SECOND32										0x2BE6
#define BT_UUID_CHARACTERISTIC_VOC_CONCENTRATION									0x2BE7
#define BT_UUID_CHARACTERISTIC_VOLTAGE_FREQUENCY									0x2BE8
#define BT_UUID_CHARACTERISTIC_BATTERY_CRITICAL_STATUS								0x2BE9
#define BT_UUID_CHARACTERISTIC_BATTERY_HEALTH_STATUS								0x2BEA
#define BT_UUID_CHARACTERISTIC_BATTERY_HEALTH_INFO									0x2BEB
#define BT_UUID_CHARACTERISTIC_BATTERY_INFO											0x2BEC
#define BT_UUID_CHARACTERISTIC_BATTERY_LEVEL_STATUS									0x2BED
#define BT_UUID_CHARACTERISTIC_BATTERY_TIME_STATUS									0x2BEE
#define BT_UUID_CHARACTERISTIC_ESTIMATED_SERVICE_DATE								0x2BEF
#define BT_UUID_CHARACTERISTIC_BATTERY_ENERGY_STATUS								0x2BF0
#define BT_UUID_CHARACTERISTIC_OBSERVATION_SCHED_CHANGED							0x2BF1
#define BT_UUID_CHARACTERISTIC_CURRENT_ELAPSED_TIME									0x2BF2
#define BT_UUID_CHARACTERISTIC_HEALTH_SENSOR_FEATURES								0x2BF3
#define BT_UUID_CHARACTERISTIC_GHS_CONTROL_POINT									0x2BF4
#define BT_UUID_CHARACTERISTIC_LE_GATT_SECURITY_LEVELS								0x2BF5
#define BT_UUID_CHARACTERISTIC_ESL_ADDRESS											0x2BF6
#define BT_UUID_CHARACTERISTIC_AP_SYNC_KEY_MATERIAL									0x2BF7
#define BT_UUID_CHARACTERISTIC_ESL_RESP_KEY_MATERIAL								0x2BF8
#define BT_UUID_CHARACTERISTIC_ESL_CURRENT_ABSOLUTE_TIME							0x2BF9
#define BT_UUID_CHARACTERISTIC_ESL_DISPLAY_INFO										0x2BFA
#define BT_UUID_CHARACTERISTIC_ESL_IMAGE_INFO										0x2BFB
#define BT_UUID_CHARACTERISTIC_ESL_SENSOR_INFO										0x2BFC
#define BT_UUID_CHARACTERISTIC_ESL_LED_INFO											0x2BFD
#define BT_UUID_CHARACTERISTIC_ESL_CONTROL_POINT									0x2BFE
#define BT_UUID_CHARACTERISTIC_UDI_MEDICAL_DEVICES									0x2BFF
#define BT_UUID_CHARACTERISTIC_GMAP_ROLE											0x2C00
#define BT_UUID_CHARACTERISTIC_UGG_FEATURES											0x2C01
#define BT_UUID_CHARACTERISTIC_UGT_FEATURES											0x2C02
#define BT_UUID_CHARACTERISTIC_BGS_FEATURES											0x2C03
#define BT_UUID_CHARACTERISTIC_BGR_FEATURES											0x2C04

// 3.9 Object Types
#define BT_UUID_OBJ_TYPE_UNSPECIFIED												0x2ACA
#define BT_UUID_OBJ_TYPE_DIRECTORY_LISTING											0x2ACB

// 3.10 SDO Services
#define BT_UUID_SDO_SERVICE_ALIRO_SERVICE											0xFFF2	// Connectivity Standard Alliance
#define BT_UUID_SDO_SERVICE_FIRA_CONSORTIUM1                   						0xFFF3
#define BT_UUID_SDO_SERVICE_FIRA_CONSORTIUM2                   						0xFFF4
#define BT_UUID_SDO_SERVICE_CAR_CONNECTIVITY_CONSORTIUM        						0xFFF5
#define BT_UUID_SDO_SERVICE_ZIGBEE_ALLIANCE1                   						0xFFF6
#define BT_UUID_SDO_SERVICE_ZIGBEE_ALLIANCE2                   						0xFFF7
#define BT_UUID_SDO_SERVICE_MOPIA_ALLIANCE                     						0xFFF8
#define BT_UUID_SDO_SERVICE_FAST_IDENTITY_ONLINE_ALLIANCE1     						0xFFF9
#define BT_UUID_SDO_SERVICE_ASTM_INTERNATIONAL                 						0xFFFA
#define BT_UUID_SDO_SERVICE_THREAD_GROUP_INC                   						0xFFFB
#define BT_UUID_SDO_SERVICE_AIRFUEL_ALLICANCE1                 						0xFFFC
#define BT_UUID_SDO_SERVICE_FAST_IDENTITY_ONLINE_ALLIANCE2     						0xFFFD
#define BT_UUID_SDO_SERVICE_AIRFUEL_ALLICANCE2                 						0xFFFE

// 3.11 Member Services
#define BT_UUID_MEMBER_SERVICE_JAPAN_DISPLAY										0xFC97
#define BT_UUID_RUUVI_INNOVATION_LTD												0xFC98

// 3.12 Mesh Profile
#define BT_UUID_MESH_PROFILE_AMBIENT_LIGHT_SENSOR									0x1600
#define BT_UUID_MESH_PROFILE_BASIC_LIGHTNESS_CONTROLLER								0x1601
#define BT_UUID_MESH_PROFILE_BASIC_SCENE_SELECTOR									0x1602
#define BT_UUID_MESH_PROFILE_DIMMING_CONTROL										0x1603
#define BT_UUID_MESH_PROFILE_ENERGY_MONITOR											0x1604
#define BT_UUID_MESH_PROFILE_OCCUPENCY_SENSOR										0x1605


// Bluetooth SIG base uuid
// 00000000-0000-1000-8000-00805F9B34FB
#define BLUETOOTH_SIG_BASE_UUID		{ 0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
									  0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }


#pragma pack(push, 1)

typedef enum __Bt_Uuid_Type {
	BT_UUID_TYPE_16,			//!< 16 bits UUID, require base UUID
	BT_UUID_TYPE_32,			//!< 32 bits UUID, require base UUID
	BT_UUID_TYPE_128			//!< Full 128 UUID
} BT_UUID_TYPE;

typedef union __Bt_Uuid_Val {
	uint16_t Uuid16;			//!< 16 bits UUID to use with base UUID defined by BaseIdx
	uint32_t Uuid32;			//!< 32 bits UUID to use with base UUID defined by BaseIdx
	uint8_t Uuid128[16];		//!< Full 128 bits UUID
} BtUuidVal_t;

typedef struct __Bt_Uuid {
	uint8_t BaseIdx:6;			//!< Base UUID index 0 - Bluetooth std, 1.. custom base
	BT_UUID_TYPE Type:2;		//!< UUID type 16, 32 or full 128 bits
	union {
		uint16_t Uuid16;
		uint32_t Uuid32;
	};
} BtUuid_t;

typedef struct __Bt_Uuid16 {
	uint8_t BaseIdx:6;			//!< Base UUID index 0 - Bluetooth std, 1.. custom base
	BT_UUID_TYPE Type:2;		//!< UUID type 16, 32 or full 128 bits
	uint16_t Uuid;
} BtUuid16_t;

typedef struct __Bt_Uuid32 {
	uint8_t BaseIdx:6;			//!< Base UUID index 0 - Bluetooth std, 1.. custom base
	BT_UUID_TYPE Type:2;		//!< UUID type 16, 32 or full 128 bits
	uint32_t Uuid;
} BtUuid32_t;

/// NOTE: Variable length structure
typedef struct __Bt_Uuid_Array {
	uint8_t BaseIdx:6;			//!< Base UUID index 0 - Bluetooth std, 1.. custom base
	BT_UUID_TYPE Type:2;		//!< UUID type 16, 32 or full 128 bits
	int Count;					//!< Number of elements in array
	union {
		uint16_t Uuid16[1];
		uint32_t Uuid32[1];
		uint8_t Uuid128[1][16];
	};
} BtUuidArr_t;

#if 0
typedef struct __Bt_Srvc_Uuid {
	uint16_t    uuid; /**< 16-bit UUID value or octets 12-13 of 128-bit UUID. */
	uint8_t     type; /**< UUID type, see @ref BLE_UUID_TYPES. If type is @ref BLE_UUID_TYPE_UNKNOWN, the value of uuid is undefined. */
} BtSrvcUuid_t;
#endif

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Find index of stored base UUID 128 bits
 *
 * This function lookup the internal table for a 128 bits UUID.
 *
 * @param 	Uuid : The 128 bits UUID to find
 *
 * @return	The index in the internal UUID table
 * 			-1 : Not found
 */
int BtUuidFindBase(uint8_t const Uuid[16]);

/**
 * @brief	Add new 128 bit base UUID into the internal table
 *
 * If UUID already exits, returns its index. Otherwise add to available slot.
 *
 * @param	Uuid :	128 bits UUID to add
 *
 * @return	The index in the internal UUID table
 * 			-1 : Table full, cannot add
 */
int BtUuidAddBase(uint8_t const Uuid[16]);

/**
 * @brief	Get the base 128 bits UUID
 *
 * Find and return to base 128 UUID located at index location in the internal table
 *
 * @param	Idx		: Index into the base UUID table.
 * @param 	Uuid 	: Buffer to store the UUID found
 *
 * @return	true - UUID found
 * 			false - UUID not found
 */
bool BtUuidGetBase(int Idx, uint8_t Uuid[16]);

/**
 * @brief	Convert the BtUuid_t type to 128 bits UUID
 *
 * @param 	pUuid	: Reference to the BtUuid_t to convert
 * @param 	Uuid128	: Buffer to store the 128 bits UUID result
 *
 * @return	true - success
 */
bool BtUuidTo128(BtUuid_t * const pUuid, uint8_t Uuid128[16]);

/**
 * @brief	Convert the BtUuid16_t 16 bits type to 128 bits UUID
 *
 * @param 	pUuid	: Reference to the BtUuid16_t to convert
 * @param 	Uuid128	: Buffer to store the 128 bits UUID result
 *
 * @return	true - success
 */
bool BtUuid16To128(BtUuid16_t * const pUuid, uint8_t Uuid128[16]);

/**
 * @brief	Convert the BtUuid32_t 32 bits type to 128 bits UUID
 *
 * @param 	pUuid	: Reference to the BtUuid32_t to convert
 * @param 	Uuid128	: Buffer to store the 128 bits UUID result
 *
 * @return	true - success
 */
bool BtUuid32To128(BtUuid32_t * const pUuid, uint8_t Uuid128[16]);

/**
 * 	Check if the base UUID128 of a given 128-bit UUID was already in the internal UUID table
 * 	If not, add the base UUID128 of this 128-bit UUID into the the table
 * 	Extract the 16-bit UUID (byte 12 and 13) from the given 128-bit UUID
 *
 * @param pUuid16		: Pointer to the 16-bit UUID
 * @param Uuid128		: The input 128-bit UUID
 *
 * @return
 * 		The index of the base UUID128 in the internal UUID table
 */
int BtUuid128To16(BtUuid16_t *pUuid16, uint8_t Uuid128[16]);

#ifdef __cplusplus
}
#endif

/** @} */

#endif // __BT_UUID_H__
