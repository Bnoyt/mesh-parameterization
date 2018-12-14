# Mesh Parameterization

Implémentation d'un algorithme de paramétrage de surfaces venant de :

	Least Squares Conformal Maps
	for Automatic Texture Atlas Generation
	Bruno Lévy
	Sylvain Petitjean
	Nicolas Ray
	Jérome Maillot
	ISA (Inria Lorraine and CNRS), France

## Installation des dépendances

Ce projet marche en python3, mais il faut vérifier l'installation de quelques dépendances pour que le projet puisse s'exécuter :


	pip3 install -r requirements.txt


## Lancement du code

Il suffit d'exécuter le fichier main.py via : 

	python3 main.py


Il est possible de lancer tous les fichiers .ply du répertoire (mais plus ils sont gros, plus le calcul prend du temps)
Il suffit alors de modifier l'une des premières lignes du fichier main.py :

	mesh = TriangularMesh(read_triangle_mesh("bun_zipper_res4.ply"))

et de remplacer bun_zipper_res4.ply par ce que vous désirez.




