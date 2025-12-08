---
description: "Comprendre les anti-patterns RxJS pour écrire du code plus robuste et maintenable. Explore systématiquement les problèmes fréquents et leurs solutions : mauvaise utilisation des Subjects, souscriptions imbriquées, branchements conditionnels dans subscribe, prolifération de drapeaux, et autres problèmes rencontrés sur le terrain."
---

# Collection d'Anti-patterns RxJS

RxJS est une bibliothèque puissante pour la programmation réactive, mais une utilisation incorrecte peut créer un terrain fertile pour les bugs et diminuer la maintenabilité. Cette section présente les erreurs courantes lors de l'utilisation de RxJS avec TypeScript et les meilleures pratiques pour les éviter.

## Objectif de cette section

- **Prévenir les bugs** : Comprendre les erreurs courantes à l'avance pour éviter les problèmes lors de l'implémentation
- **Améliorer la maintenabilité** : Maîtriser des patterns de code lisibles et testables
- **Optimiser les performances** : Apprendre des techniques pour prévenir les fuites mémoire et les traitements inutiles

## Liste des anti-patterns

Cette section couvre 17 anti-patterns.

### 🔴 Problèmes critiques

Ces patterns peuvent avoir un impact sérieux sur votre application.

| Pattern | Problème | Impact |
|---|---|---|
| **[Exposition externe des Subjects](./common-mistakes#1-subject-の外部公開)** | Exposer directement un `Subject` permettant aux externes d'appeler `next()` | Imprévisibilité de la gestion d'état, difficulté de débogage |
| **[Souscriptions imbriquées](./common-mistakes#2-ネストした-subscribe-コールバック地獄)** | Appeler `subscribe` à l'intérieur d'un autre `subscribe` | Callback hell, complexité de gestion d'erreurs |
| **[Prolifération de drapeaux de gestion d'état](./flag-management)** | Gérer l'état avec 17 drapeaux booléens, persistance de pensée impérative | Faible lisibilité, maintenance difficile, source de bugs |
| **[Imbrication if dans subscribe](./subscribe-if-hell)** | Branchements conditionnels complexes dans `subscribe` (3+ imbrications) | Faible lisibilité, tests difficiles, contraire à la pensée déclarative |
| **[Oubli d'unsubscribe](./common-mistakes#3-unsubscribe-忘れ-メモリリーク)** | Ne pas se désabonner des flux infinis | Fuites mémoire, gaspillage de ressources |
| **[Mauvaise utilisation de shareReplay](./common-mistakes#4-sharereplay-の誤用)** | Utiliser `shareReplay` sans comprendre son fonctionnement | Référence à des données périmées, fuites mémoire |

### 🟡 Problèmes nécessitant attention

Ceux-ci peuvent poser problème dans certaines situations spécifiques.

| Pattern | Problème | Impact |
|---|---|---|
| **[Effets de bord dans map](./common-mistakes#5-map-での副作用)** | Modifier l'état dans l'opérateur `map` | Comportement imprévisible, tests difficiles |
| **[Ignorer Cold/Hot](./common-mistakes#6-cold-hot-observable-の違いの無視)** | Ne pas considérer la nature de l'Observable | Exécutions en double, comportement inattendu |
| **[Mélange avec Promises](./promise-observable-mixing)** | Ne pas convertir correctement entre Promise et Observable | Non-annulable, gestion d'erreurs insuffisante |
| **[Ignorer le backpressure](./common-mistakes#8-バックプレッシャーの無視)** | Négliger le contrôle des événements à haute fréquence | Dégradation des performances, gel de l'UI |

### 🔵 Problèmes de qualité de code

Ceux-ci ne sont pas des bugs directs mais réduisent la qualité du code.

| Pattern | Problème | Impact |
|---|---|---|
| **[Suppression d'erreurs](./common-mistakes#9-エラーの握りつぶし)** | Ne pas gérer correctement les erreurs | Débogage difficile, expérience utilisateur dégradée |
| **[Fuites d'événements DOM](./common-mistakes#10-dom-イベントサブスクリプションのリーク)** | Ne pas libérer les écouteurs d'événements DOM | Fuites mémoire, dégradation des performances |
| **[Manque de sécurité de type](./common-mistakes#11-型安全性の欠如-any-の多用)** | Utilisation excessive de `any` | Erreurs au runtime, refactoring difficile |
| **[Sélection d'opérateur inappropriée](./common-mistakes#12-不適切なオペレーター選択)** | Utiliser un opérateur inadapté à l'objectif | Inefficacité, comportement inattendu |
| **[Complexification excessive](./common-mistakes#13-過度な複雑化)** | Complexifier des traitements qui pourraient être simples | Faible lisibilité, maintenance difficile |
| **[Enfer du one-liner](./one-liner-hell)** | Mélange de définition de flux, transformation et souscription | Débogage difficile, tests difficiles, faible lisibilité |
| **[Modification d'état dans subscribe](./common-mistakes#14-subscribe-内での状態変更)** | Modifier directement l'état dans `subscribe` | Tests difficiles, source de bugs |
| **[Absence de tests](./common-mistakes#15-テストの欠如)** | Ne pas écrire de tests pour le code RxJS | Régression, refactoring difficile |

## Comment progresser dans l'apprentissage

1. Étudier en détail les 15 anti-patterns dans **[Erreurs courantes et solutions](./common-mistakes)**
2. Chaque anti-pattern comprend des exemples de code "mauvais" et "bons"
3. Réviser votre code avec la **[Checklist d'évitement des anti-patterns](./checklist)**
4. Mettre en pratique les meilleures pratiques et les partager au sein de l'équipe

## Sections connexes

Après avoir appris les anti-patterns, consultez également les sections suivantes.

- **[Gestion des erreurs](/fr/guide/error-handling/strategies)** - Stratégies appropriées de gestion d'erreurs
- **[Méthodes de test](/fr/guide/testing/unit-tests)** - Comment tester le code RxJS
- **[Comprendre les opérateurs](/fr/guide/operators/)** - Comment choisir les bons opérateurs

## Prochaines étapes

1. Commencez par **[Erreurs courantes et solutions](./common-mistakes)** pour apprendre les anti-patterns pratiques et leurs solutions.
2. Après l'apprentissage, révisez votre code actuel avec la **[Checklist d'évitement des anti-patterns](./checklist)**.

---

**Important** : Ces anti-patterns sont fréquemment rencontrés dans les projets réels. Les comprendre tôt vous permettra d'écrire du code RxJS de haute qualité.
