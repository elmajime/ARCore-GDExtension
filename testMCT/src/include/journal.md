Dans _resume :
- si on appel des fonctions d'ARCore alors la classe n'est plus définie

Lors du chargement du projet ou lancement sur Android j'ai une erreur concernant le fichier TestMCT.gdextension -> problème de chemin vers les librairies.

Pour une raison que j'ignore il a besoin d'avoir l'addon dans 
res.//android/test/assets/addons/TestMCT mais en fait il a aussi besoin de les avoirs dans res.//addons/TestMCT. Sauf que si les symboles apparaissent deux fois alors il n'est pas content...

Il va certainement falloir extraire les librairies ARCore du fichie AAR comme l'indique ce lien
https://developers.google.com/ar/develop/c/enable-arcore?hl=fr