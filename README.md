# TP Grasping

L'objectif de ce TP est de permettre à un robot (ici le [Toyota HSR](https://developer.nvidia.com/embedded/community/reference-platforms/toyota-hsr)) d'attraper des objets avec sa main robotique.

Pour ce faire nous utiliserons les caméras du robot afin de détecter l'objet puis à l'aide de bibliothèque comme [GPD](https://github.com/atenpas/gpd) nous calculerons les mouvements du bras robotique.

---

## Préambule 1 : Docker

---

### Descriptif

`Docker` est un logiciel libre permettant de lancer des applications dans des conteneurs. Il ne s'agit pas de virtualisation, mais de conteneurisation, une forme plus légère qui s'appuie sur certaines parties de la machine hôte pour son fonctionnement.

### Pourquoi utiliser docker

Pour effectuer la simulation, nous allons utiliser `ROS` ainsi que `Gazebo` un logiciel de simulation de robot. Cependant, nous utiliserons des versions spécifiques de ces logiciels qui ne fonctionnent que sur une version spécifique d'`Ubuntu`. Nous allons donc créer des conteneurs docker pour exécuter la simulation sans avoir à modifier le système d'exploitation de notre ordinateur.

---

## Simulation

---

### Mise en place de l'environnement

Il Faut premièrement [installer docker](https://docs.docker.com/get-docker/). Il est conseillé d'ajouter votre utilisateur au groupe docker afin de ne pas avoir à exécuter les commandes docker en root :

```bash
sudo usermod -aG docker utilisateur
```

Il faut maintenant que vous relanciez votre session.

Téléchargez les sous-modules :

```bash
git submodule init
git submodule update
```

Ceci va télécharger le simulateur, maintenant téléchargez l'image docker puis lancez vos conteneurs :

```bash
cd hsrb_robocup_dspl_docker
docker-compose pull

docker-compose up
```

Une fois vos conteneurs démarré vous devriez pouvoir vous connecter avec votre navigateur internet sur :

- [http://localhost:3000/](http://localhost:3000/) où vous retrouverez gazebo

- [http://localhost:3001/](http://localhost:3001/) où vous retrouverez visual studio code afin de développer votre application, mais aussi d'intéragir avec le conteneur en utilisant le terminal intégré

---

## Principe de fonctionnement

---

Voici dans les grandes lignes comment le système de grasping fonctionne :

- On analyse l'image de la caméra avec yolo pour connaître la bounding box de l'objet

- On transmet cette bounding box ainsi qu'un nuage de points au service de saisie

- On "crop" le nuage de points pour ne garder que les points à l'intérieur de la bounding box

- On nettoie le nuage de points

- On calcule des configurations de saisie pour attrapper l'objet grâce à GPD (Grasp Pose Detection)

- De cette configuration on calcule les positions succécives du bras du robot

- On retourne les configurations de saisies et les positions à la node cliente

- On applique la configuration de saisie calculées par le service de saisie sur le bras du robot pour attrapper l'objet

Pour effectuer des opérations sur les nuages de points nous utiliserons PCL (Point Cloud Librairy) qui contient tous les outils dont nous aurons besoin.

Cependant, GPD et PCL sont des bibliothèques C++, le service de saisie sera donc écrit en C++.

Pour commencer créez votre `package ROS` :

```bash
# Dans le conteneur
cd src
catkin_create_pkg grasping_tp rospy roscpp
```

---

## Analyse d'image

---

La `node ROS` qui permettra l'analyse d'image fonctionnera grâce à [yolo](https://github.com/AlexeyAB/darknet) un réseau de neurone capable d'identifier des objets sur une image. Nous utiliserons plus particulièrement `tiny yolo` qui est une version plus légère, mais aussi un peu moins précise que `yolo`.

### Compiler yolo

Tout d'abord compilez `yolo`:

```bash
cd hsrb_robocup_dspl_docker/src
mkdir dependencies
cd dependencies
git clone https://github.com/AlexeyAB/darknet.git
cd darknet
```

Maintenant modifiez le fichier `Makefile` afin d'activer les options dont nous avons besoin :

- `LIBSO=1` afin d'obtenir un fichier `.so` qui nous permet d'utiliser darknet depuis une autre application

- Si vous utilisez une carte graphique nvidia :

- `GPU=1`

- `CUDNN=1`

- `CUDNN_HALF=1`

Vous êtes maintenant près à lancer la compilation

```bash
# Dans le conteneur
cd src/dependencies/darknet/
make clean
make
```

### Service ROSCette node sera en fait un [service ROS](http://wiki.ros.org/Services), c'est-à-dire une sorte de serveur auquel d'autre node peuvent se connecter pour obtenir des informations.

Ici la node qui gérera le système de saisie se connectera sur la node de détection d'objet pour fonctionner.

La première étape est donc de créer les [messages ROS](http://wiki.ros.org/msg) qui nous permettrons de communiquer de manière standardisée entre les différentes node.

Par exemple voici le type de message que renverra notre service :

```
# This message stores the detected objects

# The list of object names.
std_msgs/String[] object_names

# All the objects bounding boxes coordinates.
grasping_tp/BoundingBoxCoord[] objects_bb

# The source cloud used.
sensor_msgs/PointCloud2 cloud
```

`sensor_msgs/PointCloud2` représente un nuage de point, c'est-à-dire un ensemble de points en 3 dimensions représentant la matière détectée autour du robot. Ca nous sera utile notamment pour calculer les positions du bras du robot.

---

1. **Créez le message** `grasping_tp/BoundingBoxCoord` **qui représente le rectangle qui entoure un objet dans une image**

2. **Créez le fichier service** `object_detection` **qui prend l'heure en entrée et retourne le message plus haut**

---

Les fichiers de configurations de yolo vous sont donnés.

Voir la [doc](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv) sur la création de message et de service dans ROS. Vous êtes maintenant près à créer votre node dont voici le principe de fonctionnement :

---

1. **On attend de recevoir une requête**

2. **On récupère l'image de la caméra et le nuage de point à partir de ces topics :**

- `/hsrb/head_rgbd_sensor/rgb/image_rect_color`

- `/hsrb/head_rgbd_sensor/depth_registered/rectified_points`

3. **On convertit l'image ROS en image opencv**

4. **On fait analyser l'image par `yolo`**

5. **On calcule les positions des points du nuage de points**

6. **On remplit le message à renvoyer**

7. **On retourne ce message**

---

Voici des ressources pouvant vous aider à créer votre node :

- [Comment écrire un service ROS](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)

- [Obtenir les informations d'un topic](https://www.programcreek.com/python/example/95407/rospy.wait_for_message)

- [Doc CV bridge](http://wiki.ros.org/cv_bridge)

- [Bridge darknet/python](https://github.com/AlexeyAB/darknet/blob/master/darknet.py)

- [Créer le message ROS à renvoyer](http://wiki.ros.org/rospy/Overview/Messages)

- [Obtenir les points d'un nuage de points](https://answers.ros.org/question/240491/point_cloud2read_points-and-then/)

---

## Saisie d'objet

---

Le module de saisie d'objet est composée elle aussi d'un service écrit en C++ qui calcule les coordonnées des positions successives du bras du robot afin de pouvoir attraper un objet. Pour ce faire il faut d'abord créé les messages ROS pour permettre la communication entre la node de saisie (qui orchestre la saisie) et le service de saisie (qui calcule les configurations de saisies).

Voici le message contenant une configuration de saisie :

```
# This message describes a 2-finger grasp configuration by its 6-DOF pose,
# consisting of a 3-DOF position and 3-DOF orientation, and the opening
# width of the robot hand.

# Position
geometry_msgs/Point position # grasp position (bottom/base center of robot hand)

geometry_msgs/Pose actual_pose # grasp position (bottom/base center of robot hand)
geometry_msgs/Pose pre_pose # grasp position (bottom/base center of robot hand)
geometry_msgs/Pose after_pose # grasp position (bottom/base center of robot hand)

# Orientation represented as three axis (R = [approach binormal axis])
geometry_msgs/Vector3 approach # grasp approach direction
geometry_msgs/Vector3 binormal # hand closing direction
geometry_msgs/Vector3 axis # hand axis

std_msgs/Float32 width # Required aperture (opening width) of the robot hand

std_msgs/Float32 score # Score assigned to the grasp by the classifier

geometry_msgs/Point sample # point at which the grasp was found
```

---

1. **Créez le message retourné par le service qui contient toutes les configurations calculées**

2. **Créez le message pour effectuer une requête de saisie (pour fonction la service de saisie a besoin d'un nuage de point et d'une bounding box)**

3. **Créez le fichier service** `detect_grasps` **qui prend une requête de saisie en entrée et retourne les configurations calculées**

---

Voir la [doc](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv) sur la création de message et de service dans ROS.

Vous êtes maintenant prêt à créer votre node de saisie dont voici le fonctionnement (réalisez pour l'instant les instructions 1 à 3, vous ferez les autres quand le service de saisie sera écrit) :

---

1. **On fait une requête au service d'analyse d'image**

2. **On fait une requête au service de saisie pour obtenir les configurations de saisies, on choisit la première configuration qui est la meilleure**

3. **On applique cette configuration**

---

Voici des ressources pour les instructions 1 à 3 :

- [Utiliser un service ROS](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)

Il faut maintenant écrire le service de saisie pour ça il faut d'abord installer les dépendances.

Installez PCL :

```bash
# Dans le conteneur
sudo apt update 
sudo apt install ros-melodic-pcl-ros
```

Installez GPD :

```bash
cd hsrb_robocup_dspl_docker/src/dependencies/
git clone https://github.com/Maelic/gpd.git
cd gpd
mkdir build && cd build
cmake ..
make -j
sudo make install
```

Utilisez le fichier CMakeLists.txt donné pour pouvoir utiliser PCL et GPD, attention vos noms pour les messages et les services sont sans doute différents des miens.

Voici le principe de fonctionnement du service de saisie :

---

1. **On attend de recevoir une demande (bounding box + nuage de point)**

2. **On "crop" le nuage de point en fonction de la bounding box**

3. **On supprime les points non défini dans le nuage de point**

4. **On utilise GPD avec le nuage de point**

5. **On calcule les positions du bras du robot en fonction des configurations retournées par GPD**

6. **On retourne les positions et les configurations**

---

Pour fonctionner GPD à besoin d'un fichier de configuration qui est donné.

Utilisez le type de nuage de point suivant qui est compatible avec GPD : `pcl::PointCloud<pcl::PointXYZRGBA>::Ptr`

Voici des ressources pouvant vous aidez à écrire le service :

- [Ecrire un service ROS en C++](http://wiki.ros.org/roscpp_tutorials/Tutorials/WritingServiceClient)

- [Importez un nuage de point dans PCL](http://docs.ros.org/en/indigo/api/pcl_conversions/html/namespacepcl.html#af662c7d46db4cf6f7cfdc2aaf4439760)

- [Extraire certain point d'un nuage de points](https://pointclouds.org/documentation/tutorials/extract_indices.html), utile pour le "cropping" (n'utilisez pas la segmentation, les points à extraire sont ceux à l'intérieur de la bounding box)

- [Exemple d'utilisation de GPD](https://github.com/atenpas/gpd/blob/master/src/detect_grasps.cpp)

- [hand.h](https://github.com/atenpas/gpd/blob/master/include/gpd/candidate/hand.h) format retourné par GPD

- [Eigen vers ROS message](https://docs.ros.org/en/api/eigen_conversions/html/eigen__msg_8h.html)

Maintenant que le service de saisie est écrit **vous pouvez terminer la node de saisie**, voici des ressources utiles :

- Le fichier `utils.py` contient de quoi faire des mouvements avec le robot, pratique pour bouger le bras du robot, serrer la main du robot, bouger la tête du robot...

- [Doc moveit](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html) pour faire bouger les articulations du robot

