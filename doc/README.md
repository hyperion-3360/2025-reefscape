# Introduction

Il s'agit du dépôt de code de notre robot Abyss pour le jeu FRC 2025 Reefscape.

# Configuration de Git et flot de travail

Le repo maître est à sur github à https://github.com/hyperion-3360/2025-reefscape, la branche **main** est protégée et seul les owners de l'organisation peut y committer directement. Les membres de l'équipe doivent se créer une branche, y travailler et lorsque leurs modifications sont complétées dans la branche et pushed sur le repo (toujours dans leur branche), l'utilisation du pull request est le mécanisme. Une fois le pull request soumis, un des owners du repo va le réviser et l'accepter si le code s'intègre bien au code base existant. Dès que le code est accepté, la branche utilisateur sera détruite (il est impératif pour les membres de se créer une nouvelle branche dès qu'ils veulent poursuivre leur travail sur le code du robot).

# Structure générale du code

Nous utilisons le command based system de Wpilib. Il s'agit d'une excellent façon de partionner le code et de travailler de manière modulaire. À moins d'exception, chaque commande doit résider dans un fichier et les sous systèmes ne doivent pas retourner de commandes (même si c'est plus facile et moins verbeux)

# Assignation des commandes sur les contrôleurs de jeu

Nouveauté cette année, nous utilisons le mode de test de la driver station pour changer l'assignation des commandes sur les contrôleurs de jeu. De cette manière le mode Teleop du robot, contient toujours l'assignation officielle communiquée aux pilotes et donne la liberté à l'équipe de programmation de travailler et valider les sous systèmes manuellement en utilisant un assignation de commandes différentes.

# Foire aux questions

Vous pouvez consulter, et maintenir!!!, la foire aux questions à l'addressse suivante: https://github.com/hyperion-3360/tutoriels. 

# Owners qui peuvent accepter les pull requests:

* Gilles
* Stéphane
* *Simon* (Git god)