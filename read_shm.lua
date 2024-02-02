-- Add the required paths
cwd = '.'                             
package.cpath = cwd.."/Lib/?.so;"..package.cpath;

local shm = require('shm')
local struct = require('struct')

-- Nama shared memory yang sama dengan yang ditulis oleh program Python
local shm_rpy = "/dev/shm/imu_bno"
local shm_gyr = "/dev/shm/gyr_bno"
local data_size = 3 * 8  -- Ukuran data (3 float64, setiap float64 memiliki ukuran 8 byte)

-- Loop untuk membaca data dari shared memory
while true do
    local success = true

    -- Buka shared memory
    local file1 = io.open(shm_rpy, "rb")
    local file2 = io.open(shm_gyr, "rb")
    if not file1 then
        print("Failed to open shared memory")
        success = false
    else
        -- Baca data dari file
        local memory_data1 = file1:read(data_size)
        local memory_data2 = file2:read(data_size)
        file1:close()
        file2:close()
        --print("Raw Data:", memory_data)

        -- Pastikan data yang dibaca memiliki panjang yang sesuai
        if not memory_data1 or not memory_data2 or #memory_data1 ~= data_size or #memory_data2 ~= data_size then
            print("Failed to read valid data from shared memory")
            --success = false
        else
           -- Unpack data from binary string only if there are no nil values
           local roll, pitch, yaw = struct.unpack('ddd', memory_data1)
           local gyr_x, gyr_y, gyr_z = struct.unpack('ddd', memory_data2)

           -- Display IMU data only if there are no nil values
           if roll ~= nil and pitch ~= nil and yaw ~= nil and gyr_x ~= nil and gyr_y ~= nil and gyr_z ~= nil then
               print(string.format("Data IMU: Roll=%.2f, Pitch=%.2f, Yaw=%.2f", roll, pitch, yaw))
               print(string.format("Data GYRO: Gyr_x=%.2f, Gyr_y=%.2f, Gyr_z=%.2f", gyr_x, gyr_y, gyr_z))
           else
               print("Received nil values. Skipping unpack.")
           end
        end

    end

    if success then
        -- Tunggu sebentar sebelum membaca kembali
        os.execute("sleep 0.1")  -- Ganti dengan nilai yang sesuai dengan kebutuhan Anda
    else
        break  -- Keluar dari loop jika ada kesalahan
    end
end

