vargs=("$@")
the_type=$1
the_path=$2

energy_tools_path="/Users/voelkerb/Documents/smartenergy"
powermeter_git_path="/Users/voelkerb/Documents/smartenergy.powermeter"
smartmeter_git_path="/Users/voelkerb/Documents/smartenergy.smartmeter"
powermeter_ota_path="/Users/voelkerb/Documents/smartenergy.powermeter/Firmware/Release/espota.py"
smartmeter_ota_path="/Users/voelkerb/Documents/smartenergy.smartmeter/Firmware/Release/espota.py"

if [ "$the_type" == "none" ]; then
    echo "Nothing to be done for distribution"
elif [ "$the_type" == "smartmeter_ota" ]; then
    echo "Using SmartMeter OTA distribution"
    /usr/bin/python3 -u "$energy_tools_path/powermeter/update.py" "$smartmeter_ota_path" "$the_path" -t "SmartMeter"
elif [ "$the_type" == "powermeter_ota" ]; then
    echo "Using PowerMeter OTA distribution"
    /usr/bin/python3 -u "$energy_tools_path/powermeter/update.py" "$powermeter_ota_path" "$the_path" -t "PowerMeter"
elif [ "$the_type" == "powermeter_git" ]; then
    echo "Copying to powermeter git..."
    cp "$the_path" "$powermeter_git_path/Firmware/LatestBuild/"
elif [ "$the_type" == "smartmeter_git" ]; then
    echo "Copying to smartmeter git..."
    cp "$the_path" "$smartmeter_git_path/Firmware/LatestBuild/"
else 
    echo "Strange distribution"
fi