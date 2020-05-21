# How to use labelme

## convert HEIF -> jpg
```bash
for file in *.HEIC; do heif-convert ${file} ${file/%.heic/}.jpg; done
```