Usage:
                -c      {0,1}           i2c channel for eeprom: 0 = local, 1 = add-on pcba
                -s      {50,51,52}      i2c slave address:
                    50 = board
                    51 = pmdu
                    52 = row
                -r                      Read operation.
                -w      {file}          write operation, requires file name
                -fru_eeprom_product_type {0, 1}, 0 = EM_FRU_EEPROM_AT24C64, 1 = EM_FRU_EEPROM_AT24C02


Write Example:
                ocs-fru -c 0 -s 50 -w filename

Read Example:
                ocs-fru  -c 0 -s 50 -r

version: 1.2
build:   0.0
