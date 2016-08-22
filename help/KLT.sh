#!/bin/bash

########################################################################
#																	   #
#							Welcome Message							   #
#																	   #
########################################################################

echo "  _  __  _       _____                                                    ";
echo " | |/ / | |     |_   _|                                                   ";
echo " | ' /  | |       | |                                                     ";
echo " | . \  | |___    | |                                                     ";
echo " |_|\_\ |_____|   |_|                                 _     _             ";
echo "    / \     _   _  | |_    ___    _ __ ___     __ _  | |_  (_)   ___      ";
echo "   / _ \   | | | | | __|  / _ \  | '_ \` _ \   / _\` | | __| | |  / __|     ";
echo "  / ___ \  | |_| | | |_  | (_) | | | | | | | | (_| | | |_  | | | (__      ";
echo " /_/___\_\  \__,_|  \__|  \___/  |_| |_| |_|  \__,_|  \__| |_|  \___| _   ";
echo " | ____| __  __  _ __     ___   _ __  (_)  _ __ ___     ___   _ __   | |_ ";
echo " |  _|   \ \/ / | '_ \   / _ \ | '__| | | | '_ \` _ \   / _ \ | '_ \  | __|";
echo " | |___   >  <  | |_) | |  __/ | |    | | | | | | | | |  __/ | | | | | |_ ";
echo " |_____| /_/\_\ | .__/  _\___| |_|  _ |_| |_| |_| |_|  \___| |_| |_|  \__|";
echo " / ___|    ___  |_|__  (_)  _ __   | |_                                   ";
echo " \___ \   / __| | '__| | | | '_ \  | __|                                  ";
echo "  ___) | | (__  | |    | | | |_) | | |_                                   ";
echo " |____/   \___| |_|    |_| | .__/   \__|                                  ";
echo "                           |_|                                            ";
sleep 2

########################################################################
#																	   #
#							Options									   #
#																	   #
########################################################################

echo
echo "Would You Like To Run all of the Experiments? ('Yes', 'No' or 'Quit')"
echo "(Estimated Time:  1hr 29min)"

read input

kt='300'

if [ "$input" == 'Yes' ]; then
	it='1 2 3 4 5 6 7 8 9 10 11 12 13'    #sequences
	jt='1 2 3 4'			#Models
	#kt='300'		#FeatureNumbers
elif [ "$input" == 'No' ]; then
	echo
	echo "What Sequences Would You Like to Run?"
	echo "Enter integers (1-7) separated by spaces"
	echo "1=MH01, 2=MH02, 3=MH03, 4=MH04, 5=MH05, 6=Desk, 7=Aerial"
	echo "8=V1_1, 9=V1_2, 10=V1_3, 11=V2_1, 12=V2_2, 13=V2_3"
	read it
	echo "$it"
	echo "What IMU Models Would You Like to Test?"
	echo "Enter integers (1-4) separated by spaces"
	echo "1=noIMU, 2=IMU, 3=velocitySwitch, 4=axisAngleIntegration"
	read jt
	echo "$jt"
	#echo "What Feature Numbers Would You Like to Test?"
	#echo "Enter integers (150, 300) separated by spaces"
	#echo "150=150 Features, 300=300 Features"
	#read kt
	#echo "$kt"
else
	echo "Exiting!"
	exit
fi
									           
########################################################################
#																	   #
#				Variables for all of the sequences					   #
#																	   #
########################################################################

Sequences[1]=MH_01
Sequences[2]=MH_02
Sequences[3]=MH_03
Sequences[4]=MH_04
Sequences[5]=MH_05
Sequences[6]=Desk
Sequences[7]=Aerial

Sequences[8]=V1_01
Sequences[9]=V1_02
Sequences[10]=V1_03
Sequences[11]=V2_01
Sequences[12]=V2_02
Sequences[13]=V2_03

/home/matthew/IMUTrack/src/klt_tracker_v1.0/configs
Settings[1]=/home/matthew/IMUTrack/src/klt_tracker_v1.0/configs/EuRoC_mh_01_c0.cfg 
Settings[2]=/home/matthew/IMUTrack/src/klt_tracker_v1.0/configs/EuRoC_mh_02_c0.cfg 
Settings[3]=/home/matthew/IMUTrack/src/klt_tracker_v1.0/configs/EuRoC_mh_03_c0.cfg 
Settings[4]=/home/matthew/IMUTrack/src/klt_tracker_v1.0/configs/EuRoC_mh_04_c0.cfg 
Settings[5]=/home/matthew/IMUTrack/src/klt_tracker_v1.0/configs/EuRoC_mh_05_c0.cfg 
Settings[6]=/home/matthew/IMUTrack/src/klt_tracker_v1.0/configs/data_desk_scene.cfg 
Settings[7]=/home/matthew/IMUTrack/src/klt_tracker_v1.0/configs/data_aerial_uav.cfg

Settings[8]=/home/matthew/IMUTrack/src/klt_tracker_v1.0/configs/EuRoC_v1_01_c0.cfg 
Settings[9]=/home/matthew/IMUTrack/src/klt_tracker_v1.0/configs/EuRoC_v1_02_c0.cfg 
Settings[10]=/home/matthew/IMUTrack/src/klt_tracker_v1.0/configs/EuRoC_v1_03_c0.cfg 
Settings[11]=/home/matthew/IMUTrack/src/klt_tracker_v1.0/configs/EuRoC_v2_01_c0.cfg 
Settings[12]=/home/matthew/IMUTrack/src/klt_tracker_v1.0/configs/EuRoC_v2_02_c0.cfg  
Settings[13]=/home/matthew/IMUTrack/src/klt_tracker_v1.0/configs/EuRoC_v2_03_c0.cfg 


Model[1]=noIMU
Model[2]=IMU
Model[3]=velocitySwitch
Model[4]=axisAngleIntegration


########################################################################
#																	   #
#						Initialization								   #
#																	   #
########################################################################

#Move to the KLT Directory
cd /home/matthew/IMUTrack/build


echo
echo "Deleting Old Log Files (hit Enter to Continue)"

read input

find LogFiles -type f -delete


#Calculate Times to let the user know How Long they Have to wait (later)
#for s in $it
#do
#	actualLengths[$s]=(${lengths[$s]}+20)*4
#	echo ${actualLengths[$s]}
#done
#
#sleep 10



########################################################################
#																	   #
#		       Loop through and Execute Each Experiment				   #
#																	   #
########################################################################

echo
for i in $it
do
	for j in $jt
	do
		for k in $kt
		do
			clear
			echo "Running ${Sequences[$i]} with ${Model[$j]} and $k features"
			echo

			sleep 0.5
			

			#Run KLT
			#x-terminal-emulator --geometry 80x24+1000+0 -e /home/matthew/klt_tracker_v1.0/bin/klt_tracker -f ${Settings[$i]} -seq $i -model $j -n $k
			./klt_tracker -f ${Settings[$i]} -seq $i -model $j -n $k
		done
	done
done

########################################################################
#																	   #
#		       	Zip up the Files and (email later?)					   #
#																	   #
########################################################################

#obtain the timestamp
timestamp=`date --rfc-3339=seconds`
j=1
for i in $timestamp
do
	#echo $i
	if [ "$j" == 1 ]; then
		echo $i
		dateStamp=$i
	elif [ "$j" == 2 ]; then
		echo $i
		theTime=$i
	fi	
	j=$((j+1))
done




echo "Done Running ALL of the Experiments! :)"
echo
echo "Zipping the folder containing all of the log files!"
zip -r KLT_LogFiles_"$dateStamp"_"$theTime".zip LogFiles

echo "Done Zipping!"
echo "Goodbye!"
exit

























