Anggota Kelompok
1. Steven Anthony - 235150201111035
2. Richard - 235150200111031
3. Malvinshah Haris Athala - 235150207111042


# Command Line 

Tabel berikut merangkum parameter *command line* yang dapat digunakan pada simulasi  

| Parameter          | Deskripsi                                                         | Nilai Default              | Ruang Nilai (Scope) |
|--------------------|-------------------------------------------------------------------|----------------------------|---------------------|
| `--numNodes`       | Jumlah node ZigBee dalam jaringan                                 | `6`                        | Integer, minimum 3  |
| `--simTime`        | Lama simulasi (dalam detik)                                       | `300`                      | > 0 detik           |
| `--channel`        | Channel ZigBee yang digunakan (11–26)                             | `15`                       | 11–26               |
| `--verbose`        | Mengaktifkan log detail (LOG_LEVEL_DEBUG)                         | `false`                    | `true` / `false`    |
| `--exportCSV`      | Menyimpan data sensor dan statistik jaringan ke berkas CSV        | `true`                     | `true` / `false`    |
| `--exportXml`      | Menyimpan ringkasan jaringan ke berkas XML                        | `true`                     | `true` / `false`    |
| `--enableNetAnim`  | Menghasilkan file animasi untuk visualisasi di NetAnim           | `true`                     | `true` / `false`    |
| `--csvFile`        | Nama file output CSV (sensor + statistik per node)                | `zigbee_sensor_data.csv`   | String              |
| `--xmlFile`        | Nama file output XML ringkasan jaringan                           | `zigbee_network.xml`       | String              |
| `--animFile`       | Nama file output animasi NetAnim                                  | `zigbee_animation.xml`     | String              |
