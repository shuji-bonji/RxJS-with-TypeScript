---
description: "Les opérateurs de transformation RxJS sont utilisés pour traiter et transformer les données dans un flux. Des transformations simples aux transformations asynchrones, en passant par la mise en mémoire tampon et les modèles de fenêtrage avec map, scan, mergeMap, switchMap, concatMap et plus encore, expliqués avec des exemples pratiques TypeScript tirant parti de la sécurité des types."
---

# Opérateurs de transformation

Les opérateurs de transformation sont utilisés pour transformer et traiter les données dans un pipeline RxJS.
En convertissant les valeurs en de nouvelles formes, vous pouvez contrôler les flux de données réactifs de manière plus souple et plus puissante.

## Liste des opérateurs

### ◾ Transformation de valeur simple

| Opérateur | Description |
|---|---|
| [map](./map) | Appliquer une fonction de transformation à chaque valeur |

### ◾ Traitement d'accumulation

| Opérateur | Description |
|---|---|
| [scan](./scan) | Génère des valeurs de manière accumulative |
| [reduce](./reduce) | Ne produit que le résultat final accumulé |

### ◾ Traitement des paires/regroupements

| Opérateur | Description |
|---|---|
| [pairwise](./pairwise) | Traitement des valeurs consécutives par paires |
| [groupBy](./groupBy) | Regroupement des valeurs sur la base d'une clé |

### ◾ Transformation asynchrone

| Opérateur | Description |
|---|---|
| [mergeMap](./mergeMap) | Convertir chaque valeur en Observable et fusionner en parallèle |
| [switchMap](./switchMap) | Passage à l'Observable le plus récent |
| [concatMap](./concatMap) | Exécuter chaque Observable séquentiellement |
| [exhaustMap](./exhaustMap) | Ignorer les nouvelles entrées pendant le traitement |
| [expand](./expand) | Développer les résultats de manière récursive |

### ◾ Traitement par lots

| Opérateur | Description |
|---|---|
| [buffer](./buffer) | Collecter des valeurs au timing d'un autre Observable |
| [bufferTime](./bufferTime) | Collecte de valeurs à intervalles de temps fixes |
| [bufferCount](./bufferCount) | Collecte à un nombre spécifié |
| [bufferWhen](./bufferWhen) | Buffer avec contrôle dynamique de la condition de fin |
| [bufferToggle](./bufferToggle) | Buffer avec contrôle indépendant du début et de la fin |
| [windowTime](./windowTime) | Diviser en sous-Observables à intervalles de temps fixes |

## Modèles de transformation pratiques

Dans les applications du monde réel, la combinaison d'opérateurs de transformation permet :

- Validation des entrées et retour d'information
- Contrôle optimal des requêtes API asynchrones
- Formatage, agrégation et normalisation des données
- Traitement par lots et regroupement de flux d'événements

👉 Voir : [Modèles de transformation pratiques](./practical-use-cases) pour plus de détails.

## 🚨 Attention

Pour éviter les erreurs courantes lors de l'utilisation des opérateurs de transformation, veuillez également consulter :

- **[Effets secondaires dans map](/fr/guide/anti-patterns/common-mistakes#5-effets-secondaires-dans-map)** - Utiliser map comme une fonction pure
- **[Sélection inappropriée des opérateurs](/fr/guide/anti-patterns/common-mistakes#12-sélection-inappropriée-des-opérateurs)** - Utilisation correcte des opérateurs d'ordre supérieur
