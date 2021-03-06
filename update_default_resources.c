
#ifdef MBED_CLOUD_CLIENT_USER_CONFIG_FILE
#include MBED_CLOUD_CLIENT_USER_CONFIG_FILE
#endif

#include <stdint.h>

#ifdef MBED_CLOUD_DEV_UPDATE_ID
const uint8_t arm_uc_vendor_id[] = {
    0x92, 0xcc, 0x10, 0xb6, 0xab, 0x40, 0x5e, 0x30, 0x88, 0x6f, 0xa5, 0xcc, 0xc3, 0x22, 0xec, 0x24
};
const uint16_t arm_uc_vendor_id_size = sizeof(arm_uc_vendor_id);

const uint8_t arm_uc_class_id[]  = {
    0xb2, 0x12, 0xef, 0xa1, 0xf1, 0xa0, 0x58, 0x7e, 0x9f, 0x34, 0xdc, 0x26, 0x5a, 0x1c, 0x58, 0xd5
};
const uint16_t arm_uc_class_id_size = sizeof(arm_uc_class_id);
#endif

#ifdef MBED_CLOUD_DEV_UPDATE_CERT
const uint8_t arm_uc_default_fingerprint[] =  {
    0x80, 0x13, 0x12, 0xb1, 0x6d, 0x89, 0xa9, 0xa2, 0x49, 0x2e, 0x49, 0x7b, 0x55, 0x10, 0x44, 0xa8,
    0xa8, 0xb1, 0x50, 0x8d, 0xa2, 0xa2, 0x3e, 0xd5, 0x20, 0x93, 0xfc, 0x3c, 0x1d, 0x6e, 0x97, 0x43
};
const uint16_t arm_uc_default_fingerprint_size =
    sizeof(arm_uc_default_fingerprint);

const uint8_t arm_uc_default_subject_key_identifier[] =  {
};
const uint16_t arm_uc_default_subject_key_identifier_size =
    sizeof(arm_uc_default_subject_key_identifier);

const uint8_t arm_uc_default_certificate[] = {
    0x30, 0x82, 0x01, 0x4c, 0x30, 0x81, 0xf4, 0xa0, 0x03, 0x02, 0x01, 0x02, 0x02, 0x14, 0xac, 0xb2,
    0xfc, 0x86, 0xb6, 0x86, 0x41, 0x2c, 0x2e, 0x5a, 0x81, 0xad, 0x1d, 0x56, 0x41, 0x47, 0xc5, 0xef,
    0x16, 0x26, 0x30, 0x0a, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x04, 0x03, 0x02, 0x30, 0x14,
    0x31, 0x12, 0x30, 0x10, 0x06, 0x03, 0x55, 0x04, 0x03, 0x0c, 0x09, 0x6c, 0x6f, 0x63, 0x61, 0x6c,
    0x68, 0x6f, 0x73, 0x74, 0x30, 0x1e, 0x17, 0x0d, 0x31, 0x38, 0x30, 0x39, 0x32, 0x31, 0x32, 0x31,
    0x35, 0x32, 0x31, 0x32, 0x5a, 0x17, 0x0d, 0x31, 0x39, 0x30, 0x39, 0x32, 0x31, 0x32, 0x31, 0x35,
    0x32, 0x31, 0x32, 0x5a, 0x30, 0x14, 0x31, 0x12, 0x30, 0x10, 0x06, 0x03, 0x55, 0x04, 0x03, 0x0c,
    0x09, 0x6c, 0x6f, 0x63, 0x61, 0x6c, 0x68, 0x6f, 0x73, 0x74, 0x30, 0x59, 0x30, 0x13, 0x06, 0x07,
    0x2a, 0x86, 0x48, 0xce, 0x3d, 0x02, 0x01, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x03, 0x01,
    0x07, 0x03, 0x42, 0x00, 0x04, 0x58, 0xeb, 0xe2, 0x72, 0x6a, 0x69, 0xed, 0x5c, 0x63, 0xdb, 0xc4,
    0xde, 0x70, 0xf9, 0x6a, 0x34, 0x8d, 0xd1, 0xc3, 0xdc, 0xd6, 0x31, 0xd3, 0xee, 0x9f, 0x5a, 0x58,
    0x87, 0xdb, 0xb6, 0x77, 0xee, 0x21, 0x57, 0xff, 0xd0, 0x99, 0xfe, 0x27, 0x49, 0x31, 0x6a, 0x9d,
    0x12, 0x75, 0x91, 0x54, 0xee, 0x9f, 0xa8, 0x12, 0x69, 0xbd, 0x25, 0xf7, 0x59, 0x47, 0xd5, 0x94,
    0x0a, 0x2e, 0x2d, 0x76, 0x05, 0xa3, 0x24, 0x30, 0x22, 0x30, 0x0b, 0x06, 0x03, 0x55, 0x1d, 0x0f,
    0x04, 0x04, 0x03, 0x02, 0x07, 0x80, 0x30, 0x13, 0x06, 0x03, 0x55, 0x1d, 0x25, 0x04, 0x0c, 0x30,
    0x0a, 0x06, 0x08, 0x2b, 0x06, 0x01, 0x05, 0x05, 0x07, 0x03, 0x03, 0x30, 0x0a, 0x06, 0x08, 0x2a,
    0x86, 0x48, 0xce, 0x3d, 0x04, 0x03, 0x02, 0x03, 0x47, 0x00, 0x30, 0x44, 0x02, 0x20, 0xce, 0x59,
    0x0b, 0xf5, 0xae, 0x3a, 0xfb, 0x13, 0xb2, 0x74, 0x63, 0xbc, 0x21, 0xa7, 0xb2, 0x92, 0x83, 0xbb,
    0x3b, 0xab, 0xf9, 0x83, 0x93, 0x0f, 0x64, 0x88, 0xdf, 0x2a, 0xf5, 0xb2, 0x48, 0xd4, 0x02, 0x20,
    0x04, 0x7e, 0xc1, 0x74, 0x77, 0x62, 0x25, 0x40, 0x65, 0x6b, 0x83, 0xc8, 0x1a, 0xc2, 0x5f, 0xb7,
    0xcc, 0x62, 0xf1, 0xf9, 0x26, 0x54, 0xbb, 0x81, 0xd1, 0xdc, 0xf3, 0x6a, 0x90, 0xcd, 0x17, 0x8d
};
const uint16_t arm_uc_default_certificate_size = sizeof(arm_uc_default_certificate);
#endif


#ifdef MBED_CLOUD_DEV_UPDATE_PSK
const uint8_t arm_uc_default_psk[] = {

};
const uint8_t arm_uc_default_psk_size = sizeof(arm_uc_default_psk);
const uint16_t arm_uc_default_psk_bits = sizeof(arm_uc_default_psk)*8;

const uint8_t arm_uc_default_psk_id[] = {

};
const uint8_t arm_uc_default_psk_id_size = sizeof(arm_uc_default_psk_id);
#endif
