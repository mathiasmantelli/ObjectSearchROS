#!/bin/bash

OutputAddress="/home/mathias/Dropbox/Testes/"

MapAddress="/home/mathias/Phi-Exploration/mapas/"
MapName="hotel2"

StartX=45000 #duplos/iguais = 25500 | hotel1 = 78500 | hotel11 = 47000 -scale=60 | hotel2 = 46000 
StartY=62000 #duplos/iguais = 28500 | hotel1 = 49000 | hotel11 = 30000           | hotel2 = 64000 
StartTH=0

SegThres=250 #duplos/iguais = 450 | hotel11 = 450 | hotel2 = 250

Doors=( #duplos/iguais = 55, 54, 111, 124 | #hotel11 = 314, 336, 365, 359 | #hotel2 = 76, 148, 185, 135
"76"
"135"
"148"
"185"
)

Pesos_p=(
"0.0"
"0.5"
"0.6"
"0.7"
"0.8"
"0.9"
"1.0"
)

Explorations=(
#"BVP_exp"
#"BVP_MOD_exp"
"SEMANTIC_exp"
)

for d in "${!Doors[@]}"
do
door="${Doors[$d]}"

	for e in "${!Explorations[@]}"
	do
	exp="${Explorations[$e]}"

		if [ $exp = "SEMANTIC_exp" ]
		then
			mkdir -p -- $OutputAddress$exp

			for w in "${!Pesos_p[@]}"
			do
				peso_p="${Pesos_p[$w]}"
					
				mkdir $OutputAddress$exp"/Prints/"$door"_"$peso_p
				
				for i in {1..10}
				do
						mkdir $OutputAddress$exp"/Prints/"$door"_"$peso_p"/"$i
						 
						touch $OutputAddress$exp"/"$door"_"$peso_p"_"$i"_"$MapName".txt"		

						touch "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"

						echo "goal_number = "$door >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"

						echo "threshold_segment = "$SegThres >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"

						echo "exploration_kind = "$exp >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"

						if [ $peso_p = 0.0 ]
						then
							echo "w_alpha = "$peso_p >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"
							echo "w_orientation = 0" >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"
							echo "w_distanceGoal = 1" >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"
							echo "w_historyOrientation = 0" >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"
						fi
						if [ $peso_p != 0.0 ]
						then
							echo "w_alpha = "$peso_p >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"
							echo "w_orientation = 1" >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"
							echo "w_distanceGoal = 1" >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"
							echo "w_historyOrientation = 1" >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"
						fi

						echo "w_tollerance = 8" >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"

						echo "test_number = "$i >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"

						echo "min_vector_direction = 3" >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"

						echo "init_x = "$StartX >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"

						echo "init_y = "$StartY >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"

						echo "init_th = "$StartTH >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"

						echo "output_address = "$OutputAddress$exp"/"$door"_"$peso_p"_"$i"_"$MapName".txt" >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"
	
						echo "doors_file_address = "$MapAddress$MapName".doors" >> "config_"$exp"_"$door"_"$peso_p"_"$i"_"$MapName".ini"
				done
			done
		else

			mkdir -p -- $OutputAddress$exp

			touch $OutputAddress$exp"/"$door"_"$MapName".txt"		

			touch "config_"$exp"_"$door"_"$MapName".ini"

			echo "goal_number = "$door >> "config_"$exp"_"$door"_"$MapName".ini"

			echo "threshold_segment = "$SegThres >> "config_"$exp"_"$door"_"$MapName".ini"

			echo "exploration_kind = "$exp >> "config_"$exp"_"$door"_"$MapName".ini"

			echo "w_parity = 0" >> "config_"$exp"_"$door"_"$MapName".ini"

			echo "w_direction = 0" >> "config_"$exp"_"$door"_"$MapName".ini"

			echo "w_difference = 0" >> "config_"$exp"_"$door"_"$MapName".ini"

			echo "w_distance = 1" >> "config_"$exp"_"$door"_"$MapName".ini"

			echo "min_vector_direction = 2000" >> "config_"$exp"_"$door"_"$MapName".ini"

			echo "init_x = "$StartX >> "config_"$exp"_"$door"_"$MapName".ini"

			echo "init_y = "$StartY >> "config_"$exp"_"$door"_"$MapName".ini"

			echo "init_th = "$StartTH >> "config_"$exp"_"$door"_"$MapName".ini"

			echo "output_address = "$OutputAddress$exp"/"$door"_"$MapName".txt" >> "config_"$exp"_"$door"_"$MapName".ini"

			echo "doors_file_address = "$MapAddress$MapName".doors" >> "config_"$exp"_"$door"_"$MapName".ini"

		fi
	done
done
