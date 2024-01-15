import argparse
import struct
import sys
import time
import traceback
import readchar
import os

# Import pynrfjprog API module
from pynrfjprog import API, Hex

# Constants and other config values.
BANK_VALID_APP = 0x01
BANK_VALID_SD = 0xB7
BANK_VALID_BOOT = 0xAA
BANK_ERASED = 0xFE
BANK_INVALID_APP = 0xFF
DFU_SOFTDEVICE_ANY = 0xFFFE


# Define the C structures used for the signature file format.
# typedef struct
# {
#     uint32_t bank_0;          /**< Variable to store if bank 0 contains a valid application. */
#     uint32_t bank_0_crc;      /**< If bank is valid, this field will contain a valid CRC of the total image. */
#     uint32_t bank_1;          /**< Variable to store if bank 1 has been erased/prepared for new image. Bank 1 is only used in Banked Update scenario. */
#     uint32_t bank_0_size;     /**< Size of active image in bank0 if present, otherwise 0. */
#     uint32_t sd_image_size;   /**< Size of SoftDevice image in bank0 if bank_0 code is BANK_VALID_SD. */
#     uint32_t bl_image_size;   /**< Size of Bootloader image in bank0 if bank_0 code is BANK_VALID_SD. */
#     uint32_t app_image_size;  /**< Size of Application image in bank0 if bank_0 code is BANK_VALID_SD. */
#     uint32_t sd_image_start;  /**< Location in flash where SoftDevice image is stored for SoftDevice update. */
# } bootloader_settings_t;
bootloader_settings = struct.Struct('<IIIIIIII')

def crc16(data):
    """Generate the 16-bit CRC of the provided input string and return it."""
    crc = 0xFFFF
    for c in data:
        crc = (((crc >> 8) & 0xFF) | (crc << 8)) & 0xFFFF
        crc = (crc ^ (c if isinstance(c, int) else ord(c))) & 0xFFFF
        crc = (crc ^ ((crc & 0xFF) >> 4)) & 0xFFFF
        crc = (crc ^ ((crc << 8) << 4)) & 0xFFFF
        crc = (crc ^ (((crc & 0xFF) << 4) << 1)) & 0xFFFF
    return crc


def create_app_signature(data):
    """Generate a signature for the provided input data and return it.  Input
    should be a string of bytes with the input data.
    """
    settings = bootloader_settings.pack(BANK_VALID_APP,  # Bank 0
                                        crc16(data),     # Bank 0 CRC16
                                        BANK_ERASED,     # Bank 1
                                        len(data),       # Bank 0 length
                                        0,               # SD image size
                                        0,               # BL image size
                                        0,               # App image size
                                        0)               # SD image start
    return settings


# /**
#   * @brief Factory Information Configuration Registers (FICR)
#   */

# typedef struct {                                /*!< (@ 0x10000000) FICR Structure                                             */
#   __IM  uint32_t  RESERVED[4];
#   __IM  uint32_t  CODEPAGESIZE;                 /*!< (@ 0x00000010) Code memory page size                                      */
#   __IM  uint32_t  CODESIZE;                     /*!< (@ 0x00000014) Code memory size                                           */
#   __IM  uint32_t  RESERVED1[18];
#   __IM  uint32_t  DEVICEID[2];                  /*!< (@ 0x00000060) Description collection[0]: Device identifier               */
#   __IM  uint32_t  RESERVED2[6];
#   __IM  uint32_t  ER[4];                        /*!< (@ 0x00000080) Description collection[0]: Encryption Root, word
#                                                                     0                                                          */
#   __IM  uint32_t  IR[4];                        /*!< (@ 0x00000090) Description collection[0]: Identity Root, word
#                                                                     0                                                          */
#   __IM  uint32_t  DEVICEADDRTYPE;               /*!< (@ 0x000000A0) Device address type                                        */
#   __IM  uint32_t  DEVICEADDR[2];                /*!< (@ 0x000000A4) Description collection[0]: Device address 0                */
#   __IM  uint32_t  RESERVED3[21];
#   __IOM FICR_INFO_Type INFO;                    /*!< (@ 0x00000100) Device info                                                */
#   __IM  uint32_t  RESERVED4[185];
#   __IOM FICR_TEMP_Type TEMP;                    /*!< (@ 0x00000404) Registers storing factory TEMP module linearization
#                                                                     coefficients                                               */
#   __IM  uint32_t  RESERVED5[2];
#   __IOM FICR_NFC_Type NFC;                      /*!< (@ 0x00000450) Unspecified                                                */
# } NRF_FICR_Type;                                /*!< Size = 1120 (0x460)    

ficr_struct = struct.Struct('<16xII72xQ24x16x16xIQ')
ficr_address = 0x10000000
ficr_size = 1120


def parse_file(hex_file_path):
    if hex_file_path is None:
        api.close()
        raise Exception("Could not find file %s" % hex_file_path)

    # Parse the hex file with the help of the HEX module
    print('# Parsing file (%s) into segments.' % hex_file_path)
    test_program = Hex.Hex(hex_file_path)

    return test_program

def write_hex_file(api, hex_file_path):
    if hex_file_path is None:
        api.close()
        raise Exception("Could not find file %s" % hex_file_path)

    # Parse the hex file with the help of the HEX module
    print('# Parsing file (%s) into segments.' % hex_file_path)
    test_program = Hex.Hex(hex_file_path)

    # Program the parsed hex into the device's memory.
    print('# Writing %s to device.' % hex_file_path)
    for segment in test_program:
        print('Writing segment to %d' % segment.address)
        api.write(segment.address, segment.data, True)

def read_device_id(api):
    try:
        ficr_data = bytes(api.read(ficr_address, ficr_size))
        #print(ficr_data)
        codepagesize, codesize, deviceid, deviceaddresstype, deviceaddress = ficr_struct.unpack_from(ficr_data)
        #print(ficr_struct.unpack_from(ficr_data))
        return deviceid
    except:
        return None


def write_arduino_sketch(api, hex_file_path):
    if hex_file_path is None:
        api.close()
        raise Exception("Could not find example binary for device " + device_version.lower() + ".\n" +
                        "This example may not support your device yet.")

    # Parse the hex file with the help of the HEX module
    print('# Parsing hex file into segments.')
    test_program = Hex.Hex(hex_file_path)

    signature = None

    # Program the parsed hex into the device's memory.
    print('# Writing %s to device.' % hex_file_path)
    for segment in test_program:
        print('Writing segment to %d' % segment.address)
        api.write(segment.address, segment.data, True)
        signature = create_app_signature(segment.data)

    print('# Writing app signature')
    api.write(0x7f000, signature, True)

def detect_device(api, timeout=None, ignore_device_id=None):
    print("Probing for device... ", end='', flush=True)

    start_time = time.time()

    while timeout == None or (time.time() - start_time) <= timeout:

        try:
            api.connect_to_device()
            
            if api.is_connected_to_device():
                print ("ic")
                current_device_id = read_device_id(api)
                print (current_device_id)
                if current_device_id != None and current_device_id != ignore_device_id:
                    print('found {}'.format(current_device_id))
                    return current_device_id
                else:
                    api.go()
                    api.disconnect_from_device()
        except:
            pass

        time.sleep(0.5)
    
    print("failed")

    return None

def connect_to_emulator(api, timeout=None, serial_number=None):
    print("Connecting to emulator... ", end='', flush=True)

    start_time = time.time()

    final_err = None

    while timeout == None or (time.time() - start_time) <= timeout:
        try:
            if serial_number is not None:
                api.connect_to_emu_with_snr(serial_number, jlink_speed_khz=4000)
            else:
                api.connect_to_emu_without_snr(jlink_speed_khz=4000)

            print("success")
            return api.is_connected_to_emu()
        except Exception as err:
            final_err = err

        time.sleep(0.5)

    print("failed")
    print(final_err)
    return False


def run(snr=None):
    """
    Run example script.

    @param (optional) int snr: Specify serial number of DK to run example on.
    """
    print('# Memory read and write example using pynrfjprog started...')
    
    # Detect the device family of your device. Initialize an API object with UNKNOWN family and read the device's family. This step is performed so this example can be run in all devices without customer input.
    # print('# Opening API with device family UNKNOWN, reading the device family.')
    # with API.API(API.DeviceFamily.UNKNOWN) as api:            # Using with construction so there is no need to open or close the API class.
    #     if snr is not None:
    #         api.connect_to_emu_with_snr(snr)
    #     else:
    #         api.connect_to_emu_without_snr()
    #     device_family = api.read_device_family()

    snr = args.serial

    device_family = API.DeviceFamily.NRF52

    last_ficr = None

    flash_regions = []

    if args.write != None:
        for filename in args.write:
            segments = parse_file(filename)
        
            # Program the parsed hex into the device's memory.
            for segment in segments:
                flash_regions.append(segment)

    if args.write_signed != None:
        segments = parse_file(args.write_signed)

        if len(list(segments)) != 1:
            for segment in segments:
                   print('# 0x{:08x} - 0x{:08x} ({} bytes)'.format(segment.address, segment.address + segment.length, segment.length))
           # raise ValueError('Signed images may only contain one segment {} contains {}'.format(args.write_signed, len(list(segments))))
        
        # Program the parsed hex into the device's memory.
        data=[]
        for segment in segments:
            flash_regions.append(segment)
            data += segment.data
        
        signature = create_app_signature(data)
        flash_regions.append(Hex.Segment(0x7f000, signature))

    flash_regions.sort(key=lambda s: s.address)
    for segment in flash_regions:
        print('# 0x{:08x} - 0x{:08x} ({} bytes)'.format(segment.address, segment.address + segment.length, segment.length))

    if not flash_regions:
        print('Nothing to flash, exiting')
        return

    print("Opening api")
    # Initialize an API object with the target family. This will load nrfjprog.dll with the proper target family.
    api = API.API(device_family)

    # Open the loaded DLL and connect to an emulator probe. If several are connected a pop up will appear.
    api.open()

    # print("Connecting to emulator")


    # try:
    #     if args.serial is not None:
    #         api.connect_to_emu_with_snr(args.serial)
    #     else:
    #         api.connect_to_emu_without_snr()
    # except Exception as err:
    #     print(err)

    #     print("Failed to connect to JLink")
    #     return

    # print("Connected to emulator")

    last_device_id = None

    while True:
        if args.repeat and args.keypress:
            print ("Press a key to flash")
            readchar.readchar()
            last_device_id = None
                

        if not connect_to_emulator(api, 5, args.serial):
            continue


        current_device_id = detect_device(api, 5, last_device_id)

        if current_device_id == None:
            api.disconnect_from_emu()
            continue

        print('Waiting 0.5 seconds to start programming')

        time.sleep(0.5)

        try:
            # # Erase all the flash of the device.
            print('# Erasing all flash in the microcontroller.')
            api.erase_all()

            for segment in flash_regions:
                print('# Writing 0x{:08x} - 0x{:08x} ({} bytes)'.format(segment.address,
                                                        segment.address + segment.length, segment.length))
                api.write(segment.address, segment.data, True)
            
        except:
            pass
        # write_hex_file(api, 'c:\\img\\sig.hex')

        # Reset the device and run.
        api.sys_reset()
        api.go()
        print('# Application running. Your board should be blinking.')

        api.disconnect_from_device()
        api.disconnect_from_emu()


        last_device_id = current_device_id



        if not args.repeat:
            break

        print('Waiting 5 seconds before starting next cycle programming')

        time.sleep(5)

    # Close the loaded DLL to free resources.
    api.close()

    # if not args.repeat:
    #     break

    print('# Example done...')
    
 
if __name__ == '__main__':
    # Parse command line arguments.
    parser = argparse.ArgumentParser(
        description='Amulet of Rune flashing tool')
    parser.add_argument('--write',
                        action='append',
                        help='bootloader sketch',
                        metavar='FILENAME')
    parser.add_argument('--write-signed',
                        action='store',
                        help='write hex file, store signature in bootloader',
                        metavar='FILENAME')
    parser.add_argument('-q', '--quiet',
                        action='store_true',
                        help='disable all console output unless an error occurs')
    parser.add_argument('--serial',
                        type=int,
                        action='store',
                        help='programmer serial number')
    parser.add_argument('--repeat',
                        action='store_true',
                        default=False,
                        help='Repeat programming')
    parser.add_argument('--keypress',
                        action='store_true',
                        default=False,
                        help='Press a key to program programming')
    args = parser.parse_args()
    try:
        run()
    except KeyboardInterrupt:
        print('Interrupted')
        try:
            sys.exit(130)
        except SystemExit:
            os._exit(130)


