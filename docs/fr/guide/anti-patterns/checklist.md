---
description: "Checklist d'évitement des anti-patterns à vérifier lors de l'écriture de code RxJS. Couvre 16 meilleures pratiques incluant la désinscription appropriée des Subscriptions, l'utilisation correcte des Subjects, l'implémentation de la gestion d'erreurs, la prévention des fuites mémoire, et fournit les éléments essentiels pour réaliser du code réactif robuste et maintenable."
---

# Checklist d'évitement des anti-patterns

Utilisez cette checklist pour vérifier si votre code RxJS correspond à un anti-pattern. Cliquez sur chaque élément pour voir des explications détaillées et des exemples de code.

## Points de vérification

### 🔴 Éviter les problèmes critiques

| Vérification | Élément | Points clés |
|:---:|---|---|
| <input type="checkbox" /> | **[Exposer les Subjects avec asObservable()](./common-mistakes#1-subject-の外部公開)** | Ne pas exporter directement les `Subject`, les exposer comme Observable avec `asObservable()`<br>Permettre les changements d'état uniquement via des méthodes dédiées |
| <input type="checkbox" /> | **[Éviter les souscriptions imbriquées](./common-mistakes#2-ネストした-subscribe-コールバック地獄)** | Ne pas appeler d'autres `subscribe` dans un `subscribe`<br>Aplatir avec `switchMap`, `mergeMap`, `concatMap`, etc. |
| <input type="checkbox" /> | **[Toujours se désabonner des flux infinis](./common-mistakes#3-unsubscribe-忘れ-メモリリーク)** | Toujours se désabonner des flux infinis comme les écouteurs d'événements<br>Pattern `takeUntil` ou gestion de `Subscription` |
| <input type="checkbox" /> | **[Spécifier explicitement la configuration de shareReplay](./common-mistakes#4-sharereplay-の誤用)** | Utiliser le format `shareReplay({ bufferSize: 1, refCount: true })`<br>Activer le comptage de références pour prévenir les fuites mémoire |
| <input type="checkbox" /> | **[Éviter les imbrications if dans subscribe](./subscribe-if-hell)** | Éviter les branchements conditionnels complexes dans `subscribe` (3+ imbrications)<br>Écrire de manière déclarative avec des opérateurs comme `filter`, `iif`, `partition` |

### 🟡 Éviter les problèmes nécessitant attention

| Vérification | Élément | Points clés |
|:---:|---|---|
| <input type="checkbox" /> | **[map pour fonctions pures, tap pour effets de bord](./common-mistakes#5-map-での副作用)** | Ne pas modifier l'état ou logger dans `map`<br>Séparer explicitement les effets de bord avec l'opérateur `tap` |
| <input type="checkbox" /> | **[Utiliser Cold/Hot de manière appropriée](./common-mistakes#6-cold-hot-observable-の違いの無視)** | Convertir en Hot avec `shareReplay` pour les requêtes HTTP, etc.<br>Déterminer si doit être exécuté pour chaque souscription ou partagé |
| <input type="checkbox" /> | **[Convertir les Promises avec from](./common-mistakes#7-promise-と-observable-の不適切な混在)** | Ne pas mélanger Promise et Observable<br>Convertir en Observable avec `from()` pour un traitement unifié |
| <input type="checkbox" /> | **[Contrôler les événements à haute fréquence](./common-mistakes#8-バックプレッシャーの無視)** | Contrôler l'entrée de recherche avec `debounceTime`, le défilement avec `throttleTime`<br>Exclure les doublons avec `distinctUntilChanged` |

### 🔵 Améliorer la qualité du code

| Vérification | Élément | Points clés |
|:---:|---|---|
| <input type="checkbox" /> | **[Gérer correctement les erreurs](./common-mistakes#9-エラーの握りつぶし)** | Capturer les erreurs avec `catchError` et les gérer correctement<br>Afficher des messages d'erreur compréhensibles pour l'utilisateur<br>Réessayer avec `retry` / `retryWhen` si nécessaire |
| <input type="checkbox" /> | **[Libérer correctement les événements DOM](./common-mistakes#10-dom-イベントサブスクリプションのリーク)** | Toujours se désabonner des souscriptions `fromEvent`<br>Désinscription automatique avec `takeUntil` lors de la destruction du composant |
| <input type="checkbox" /> | **[Assurer la sécurité de type](./common-mistakes#11-型安全性の欠如-any-の多用)** | Définir des interfaces ou des alias de types<br>Spécifier explicitement le paramètre de type de `Observable<T>`<br>Utiliser l'inférence de type de TypeScript |
| <input type="checkbox" /> | **[Sélectionner l'opérateur approprié](./common-mistakes#12-不適切なオペレーター選択)** | Recherche: `switchMap`, parallèle: `mergeMap`<br>Séquentiel: `concatMap`, anti-spam: `exhaustMap` |
| <input type="checkbox" /> | **[RxJS non nécessaire pour traitements simples](./common-mistakes#13-過度な複雑化)** | Le JavaScript normal suffit pour le traitement de tableaux, etc.<br>Utiliser RxJS pour le traitement asynchrone et les flux d'événements |
| <input type="checkbox" /> | **[Gérer l'état de manière réactive](./common-mistakes#14-subscribe-内での状態変更)** | Gérer l'état avec `BehaviorSubject` ou `scan`<br>Utiliser `subscribe` comme déclencheur final |
| <input type="checkbox" /> | **[Écrire des tests](./common-mistakes#15-テストの欠如)** | Effectuer des tests marble avec `TestScheduler`<br>Rendre le traitement asynchrone testable de manière synchrone |

## Utilisation

### 1. Lors de la revue de code

Après avoir écrit du nouveau code, effectuez une auto-revue en utilisant cette checklist.

### 2. Lors de la pull request

En incluant cette checklist dans le template de pull request, vous pouvez vérifier avec les reviewers selon des critères communs.

### 3. Revue régulière

Utilisez régulièrement cette checklist sur la base de code existante pour vérifier qu'aucun anti-pattern ne s'est infiltré.

### 4. Partage au sein de l'équipe

Partagez avec les membres de l'équipe pour unifier les meilleures pratiques RxJS.

## Ressources connexes

- **[Erreurs courantes et solutions](./common-mistakes)** - Explication détaillée de chaque anti-pattern et exemples de code
- **[Sommaire des anti-patterns](./index)** - Liste des anti-patterns et progression de l'apprentissage
- **[Gestion des erreurs](/fr/guide/error-handling/strategies)** - Meilleures pratiques de gestion d'erreurs
- **[Méthodes de test](/fr/guide/testing/unit-tests)** - Comment tester le code RxJS

## Conseils pour utiliser la checklist

1. **Ne pas essayer de tout perfectionner en une fois**
   - Traiter d'abord prioritairement les problèmes critiques (🔴)
   - Améliorer progressivement

2. **Décider des priorités au sein de l'équipe**
   - Ajuster l'importance selon les caractéristiques du projet
   - Créer une checklist personnalisée

3. **Considérer l'automatisation**
   - Vérification automatique avec des outils d'analyse statique comme ESLint
   - Intégrer dans le pipeline CI/CD

4. **Mettre à jour régulièrement**
   - Mettre à jour selon les mises à niveau de version de RxJS
   - Refléter les connaissances acquises par l'expérience de l'équipe

---

**Important** : Cette checklist n'est pas pour écrire du code parfait, mais un guide pour éviter les problèmes courants. Utilisez-la de manière flexible selon le contexte du projet.
