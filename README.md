# Ammortizzatore "ESPID"
Autore: Stefano Arzani

## Descrizione
Il progetto consiste in un sistema di "frenata ammortizzata" di un oggetto posto su un nastro trasportatore in modo che si fermi il più vicino possibile al bordo.

## Componenti utilizzati
1 ESP32 "WROOM-32"  
1 sensore di ostacoli a ultrasuoni HC-SR04  
2 Motori DC  
1 motor driver L293D  

### Licenza
GNU General Public License v3.0

### Funzionamento
Un nastro trasportatore (in questo caso realizzato "fai da te") sarà fatto girare da due motori DC ad una certa velocità controllata dalla board; il sensore di distanza ad ultrasuoni, posto davanti al nastro, rileverà la presenza di un oggetto posto su quest'ultimo e tramite continui campionamenti di distanza la board ne calcolerà la velocità di avvicinamento e diminuirà la velocità di rotazione dei motori in modo da far fermare l'oggetto il più vicino possibile al bordo sfruttando il cosiddetto controllo PID.
