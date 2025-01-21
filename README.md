# RobotCoupe
**Librairie Arduino pour le robot de la coupe**
 ## Hardware
 - une nucleo64
 - un shield X-NUCLEO-IHM02A1
 - deux steppers

## Usecases
Le but est d'offrir une librairie simple pour contrôler le robot sans avoir à donner une instruction à chaque moteurs.
Pour l'instant, cette librairie permet de faire avancer/reculer le robot et de le faire tourner sur lui même.

## Usage
### Installation
Pour installer la librairie, aller dans le dossier du compilateur Arduino (`File` -> `Preferences` -> `Sketchbook Location`) et l'ajouter dans le dossier `librairies`.
A savoir que vous allez avoir besoin aussi de la librairie [STM32uino X-NUCLEO-IHM02A1 by STMicroelectronics][https://github.com/stm32duino/X-NUCLEO-IHM02A1]; et d'installer les cartes STMicroelectronics en copiant `https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json` dans `File` <- `Preferences` <- `Additional board manager URL`.

### Gitaneries
Il faut ensuite s'assurer que les dimensions `robotWidth`et `wheelRadius` sont correctement rentrées dans la définition des variables dans le fichier `RobotCoupe.h`.


## Example
The `triangle.ino` example is supposed to make the robot advance and turn to form a triangle of 10cm.
