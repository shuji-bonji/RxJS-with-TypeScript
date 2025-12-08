---
description: "Un guide d'apprentissage pour étudier systématiquement RxJS dans un environnement TypeScript. Il fournit des explications pratiques, étape par étape, qui couvrent tout, des principes fondamentaux des observables aux sujets, en passant par divers opérateurs, la gestion des erreurs, les planificateurs et les techniques de test."
---

# Guide

Ce guide vous aide à apprendre systématiquement RxJS dans un environnement TypeScript.
En parcourant les sections ci-dessous dans l'ordre, vous pouvez acquérir une compréhension structurée de RxJS, des principes fondamentaux aux concepts avancés.

## Table des matières

### 1. Introduction à RxJS
- [Commencer à apprendre](/fr/guide/introduction)
- [Configuration de l'environnement d'apprentissage](/fr/guide/starter-kid)
- [Qu'est-ce que RxJS ?](/fr/guide/basics/what-is-rxjs)
- [Qu'est-ce qu'un flux ?](/fr/guide/basics/what-is-a-stream)
- [Promise vs. RxJS](/fr/guide/basics/promise-vs-rxjs)

### 2. Fondamentaux des observables
- [Qu'est-ce qu'un Observable ?](/fr/guide/observables/what-is-observable)
- [Comment créer un observable](/fr/guide/observables/creation)
- [Flux d'événements](/fr/guide/observables/events)
- [Liste d'événements](/fr/guide/observables/events-list)
- [Observer vs Subscriber](/fr/guide/observables/observer-vs-subscriber)
- [Cycle de vie des observables](/fr/guide/observables/observable-lifecycle)
- [Observables froids et observables chauds](/fr/guide/observables/cold-and-hot-observables)

### 3. Fonctions de création
- [Qu'est-ce qu'une fonction de création ?](/fr/guide/creation-functions/)
- [Fonctions de création de base](/fr/guide/creation-functions/basic/) - of, from, fromEvent, interval, timer
- [Fonctions de génération de boucles](/fr/guide/creation-functions/loop/) - range, generate
- [Fonctions de communication HTTP](/fr/guide/creation-functions/http-communication/) - ajax, fromFetch
- [Fonctions de combinaison](/fr/guide/creation-functions/combination/) - concat, merge, combineLatest, zip, forkJoin
- [Fonctions de sélection et de partition](/fr/guide/creation-functions/selection/) - race, partition
- [Fonctions conditionnelles](/fr/guide/creation-functions/conditional/) - iif, defer
- [Fonctions de contrôle](/fr/guide/creation-functions/control/) - scheduled, using

### 4. Comprendre les opérateurs
- [Vue d'ensemble des opérateurs](/fr/guide/operators/)
- [Concepts de pipeline](/fr/guide/operators/pipeline)
- [Opérateurs de transformation](/fr/guide/operators/transformation/) - map, scan, mergeMap, switchMap, etc.
- [Opérateurs de filtrage](/fr/guide/operators/filtering/) - filter, take, debounceTime, throttleTime, etc.
- [Opérateurs de combinaison](/fr/guide/operators/combination/) - withLatestFrom, mergeWith, etc.
- [Opérateurs d'utilité](/fr/guide/operators/utility/) - tap, delay, retry, finalize, takeUntil, etc.
- [Multidiffusion](/fr/guide/operators/multicasting/) - share, shareReplay, etc.

### 5. Sujets et multidiffusion
- [Qu'est-ce qu'un sujet ?](/fr/guide/subjects/what-is-subject)
- [Types de sujets](/fr/guide/subjects/types-of-subject)
- [Comment fonctionne la multidiffusion](/fr/guide/subjects/multicasting)
- [Cas d'utilisation des sujets](/fr/guide/subjects/use-cases)

### 6. Traitement des erreurs
- [Stratégies de gestion des erreurs](/fr/guide/error-handling/strategies)
- [Deux emplacements pour la gestion des erreurs](/fr/guide/error-handling/error-handling-locations)
- [Intégrer try-catch avec RxJS](/fr/guide/error-handling/try-catch-integration)
- [retry et catchError](/fr/guide/error-handling/retry-catch)
- [finalize et complete](/fr/guide/error-handling/finalize)

### 7. Utilisation des planificateurs
- [Contrôle du traitement asynchrone](/fr/guide/schedulers/async-control)
- [Types d'ordonnanceurs et utilisation](/fr/guide/schedulers/types)
- [Notions de base sur les tâches et les ordonnanceurs](/fr/guide/schedulers/task-and-scheduler-basics)

### 8. Techniques de débogage RxJS
- [Aperçu des techniques de débogage](/fr/guide/debugging/)
- [Scénarios de débogage courants](/fr/guide/debugging/common-scenarios)
- [Outils de débogage personnalisés](/fr/guide/debugging/custom-tools)
- [Débogage des performances](/fr/guide/debugging/performance)

### 9. Techniques de test
- [Tests unitaires de RxJS](/fr/guide/testing/unit-tests)
- [Utilisation de TestScheduler](/fr/guide/testing/test-scheduler)
- [Test Marble](/fr/guide/testing/marble-testing)

### 10. Collection d'anti-modèles RxJS
- [Que sont les anti-modèles ?](/fr/guide/anti-patterns/)
- [Erreurs courantes et solutions](/fr/guide/anti-patterns/common-mistakes)
- [Prolifération des flags](/fr/guide/anti-patterns/flag-management)
- [Enoncés if imbriqués dans subscribe](/fr/guide/anti-patterns/subscribe-if-hell)
- [Mélange de promesses et d'observables](/fr/guide/anti-patterns/promise-observable-mixing)
- [L'enfer du one-liner](/fr/guide/anti-patterns/one-liner-hell)
- [Liste de contrôle pour éviter les anti-modèles](/fr/guide/anti-patterns/checklist)

### 11. Surmonter les difficultés de RxJS
- [Pourquoi RxJS est difficile](/fr/guide/overcoming-difficulties/)
- [L'obstacle de la compréhension conceptuelle](/fr/guide/overcoming-difficulties/conceptual-understanding)
- [L'obstacle de la gestion du cycle de vie](/fr/guide/overcoming-difficulties/lifecycle-management)
- [Dilemme de la sélection de l'opérateur](/fr/guide/overcoming-difficulties/operator-selection)
- [Comprendre la chronologie et l'ordre](/fr/guide/overcoming-difficulties/timing-and-order)
- [Difficultés liées à la gestion des états](/fr/guide/overcoming-difficulties/state-and-sharing)
- [Combinaison de flux multiples](/fr/guide/overcoming-difficulties/stream-combination)
- [Défis en matière de débogage](/fr/guide/overcoming-difficulties/debugging-guide)

### 13. Collection de modèles pratiques
- [Vue d'ensemble des modèles pratiques](/fr/guide/practical-patterns/)
- [Gestion des événements de l'interface utilisateur](/fr/guide/practical-patterns/ui-events)
- [Appels API](/fr/guide/practical-patterns/api-calls)
- [Gestion des formulaires](/fr/guide/practical-patterns/form-handling)
- [Patterns de formulaires avancés](/fr/guide/practical-patterns/advanced-form-patterns)
- [Traitement des données en temps réel](/fr/guide/practical-patterns/real-time-data)
- [Stratégies de mise en cache](/fr/guide/practical-patterns/caching-strategies)
- [Modèles de gestion des erreurs](/fr/guide/practical-patterns/error-handling-patterns)
- [Branchements conditionnels dans les abonnements](/fr/guide/practical-patterns/subscribe-branching)

### Annexe
- [Aperçu de l'annexe](/fr/guide/appendix/)
- [Développement embarqué et programmation réactive](/fr/guide/appendix/embedded-reactive-programming)
- [Modèles réactifs au-delà de RxJS](/fr/guide/appendix/reactive-patterns-beyond-rxjs)
- [Carte de l'architecture réactive](/fr/guide/appendix/reactive-architecture-map)
- [Programmation réactive reconsidérée](/fr/guide/appendix/reactive-programming-reconsidered)
- [Écosystème RxJS et Reactive Streams](/fr/guide/appendix/rxjs-and-reactive-streams-ecosystem)

---

> [!NOTE]
> Ce guide est structuré de manière à approfondir votre compréhension de RxJS étape par étape et de manière systématique. N'hésitez pas à vous référer à n'importe quelle section si nécessaire.
