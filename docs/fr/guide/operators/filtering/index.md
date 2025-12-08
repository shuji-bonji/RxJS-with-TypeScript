---
description: "Les opérateurs de filtrage RxJS extraient uniquement les données nécessaires des flux en fonction de conditions ou du temps. filter, take, skip, debounceTime, throttleTime, distinct, first, last et plus encore - fournit un guide de sélection des opérateurs par cas d'utilisation et des exemples pratiques."
---

# Opérateurs de filtrage

Les opérateurs de filtrage RxJS sont des outils essentiels pour sélectionner uniquement les données nécessaires d'un flux et empêcher les données inutiles de passer.
Cela améliore considérablement l'efficacité et les performances de l'application.

Les opérateurs de filtrage sont un groupe d'opérateurs RxJS qui sélectionnent les valeurs dans un flux et ne laissent passer que celles qui satisfont certaines conditions.
En contrôlant le flux de données et en ne traitant que les valeurs nécessaires, vous pouvez construire des pipelines de traitement de données efficaces.


## Liste des opérateurs
### ◾ Opérateurs de filtrage de base

| Opérateur | Description |
|:---|:---|
| [filter](./filter) | Ne laisse passer que les valeurs correspondant à la condition |
| [take](./take) | Récupère uniquement le premier nombre spécifié de valeurs |
| [takeLast](./takeLast) | Récupère les dernières N valeurs |
| [takeWhile](./takeWhile) | Récupère les valeurs tant que la condition est satisfaite |
| [skip](./skip) | Ignore le premier nombre spécifié de valeurs |
| [skipLast](./skipLast) | Ignore les dernières N valeurs |
| [skipWhile](./skipWhile) | Ignore les valeurs tant que la condition est satisfaite |
| [skipUntil](./skipUntil) | Ignore les valeurs jusqu'à ce qu'un autre Observable émette |
| [first](./first) | Récupère la première valeur ou la première valeur satisfaisant la condition |
| [last](./last) | Récupère la dernière valeur ou la dernière valeur satisfaisant la condition |
| [elementAt](./elementAt) | Récupère la valeur à l'index spécifié |
| [find](./find) | Trouve la première valeur satisfaisant la condition |
| [findIndex](./findIndex) | Récupère l'index de la première valeur satisfaisant la condition |
| [ignoreElements](./ignoreElements) | Ignore toutes les valeurs et ne laisse passer que complete/error |


### ◾ Opérateurs de filtrage basés sur le temps

| Opérateur | Description |
|:---|:---|
| [debounceTime](./debounceTime) | Émet la dernière valeur si aucune entrée pendant le temps spécifié |
| [throttleTime](./throttleTime) | Laisse passer la première valeur et ignore les nouvelles pendant le temps spécifié |
| [auditTime](./auditTime) | Émet la dernière valeur après le temps spécifié |
| [audit](./audit) | Émet la dernière valeur avec contrôle de période par Observable personnalisé |
| [sampleTime](./sampleTime) | Échantillonne la dernière valeur à intervalles réguliers |


### ◾ Opérateurs de filtrage basés sur les conditions

| Opérateur | Description |
|:---|:---|
| [distinct](./distinct) | Supprime toutes les valeurs dupliquées (n'émet que des valeurs uniques) |
| [distinctUntilChanged](./distinctUntilChanged) | Supprime les valeurs dupliquées consécutives |
| [distinctUntilKeyChanged](./distinctUntilKeyChanged) | Détecte uniquement les changements d'une propriété spécifique |


## Cas d'utilisation pratiques

- [Cas d'utilisation pratiques](./practical-use-cases.md) présente des exemples pratiques combinant plusieurs opérateurs de filtrage (recherche en temps réel, défilement infini, etc.).
