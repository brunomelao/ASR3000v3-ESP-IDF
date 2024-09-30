## Trabalho final Software Embarcado

Para fazer o download dos dados de voo salvos na memória flash do ESP32-s3, deve-se formar o arquivo bin com os arquivos da memoria usando o parttool.py no ESPIDF:
python /bruno/esp/v5.3/esp-idf/components/partition_table/parttool.py --port "COM15" read_partition --partition-type=data --partition-subtype=spiffs --output "littlefs.bin"

Para formatar:
python /bruno/esp/v5.3/esp-idf/components/partition_table/parttool.py --port "COM15" erase_partition --partition-type=data --partition-subtype=spiffs
