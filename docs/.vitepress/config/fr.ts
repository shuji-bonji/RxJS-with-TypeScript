import { DefaultTheme } from 'vitepress';

export const frThemeConfig: DefaultTheme.Config = {
  nav: [
    { text: 'Accueil', link: '/fr/' },
    { text: 'Guide', link: '/fr/guide/' },
  ],
  sidebar: [
    {
      text: '1. Introduction à RxJS',
      items: [
        { text: 'Introduction', link: '/fr/guide/introduction' },
        { text: "Configuration de l'environnement", link: '/fr/guide/starter-kid' },
        { text: "Qu'est-ce que RxJS ?", link: '/fr/guide/basics/what-is-rxjs' },
        { text: "Qu'est-ce qu'un Stream ?", link: '/fr/guide/basics/what-is-a-stream' },
        { text: 'Différence entre Promise et RxJS', link: '/fr/guide/basics/promise-vs-rxjs' },
      ],
    },
    {
      text: "2. Fondamentaux d'Observable",
      items: [
        { text: "Qu'est-ce qu'un Observable ?", link: '/fr/guide/observables/what-is-observable' },
        { text: 'Création d\'Observable', link: '/fr/guide/observables/creation' },
        { text: 'Streamifier les événements', link: '/fr/guide/observables/events' },
        { text: 'Liste des événements', link: '/fr/guide/observables/events-list' },
        { text: 'Observer vs Subscriber', link: '/fr/guide/observables/observer-vs-subscriber' },
        { text: 'Cycle de vie d\'Observable', link: '/fr/guide/observables/observable-lifecycle' },
        { text: 'Cold et Hot Observable', link: '/fr/guide/observables/cold-and-hot-observables' },
      ],
    },
    {
      text: '3. Fonctions de création',
      link: '/fr/guide/creation-functions/',
      items: [
        {
          text: 'Création de base',
          link: '/fr/guide/creation-functions/basic/',
          collapsed: true,
          items: [
            { text: 'of', link: '/fr/guide/creation-functions/basic/of' },
            { text: 'from', link: '/fr/guide/creation-functions/basic/from' },
            { text: 'fromEvent', link: '/fr/guide/creation-functions/basic/fromEvent' },
            { text: 'interval', link: '/fr/guide/creation-functions/basic/interval' },
            { text: 'timer', link: '/fr/guide/creation-functions/basic/timer' },
          ],
        },
        {
          text: 'Génération en boucle',
          link: '/fr/guide/creation-functions/loop/',
          collapsed: true,
          items: [
            { text: 'range', link: '/fr/guide/creation-functions/loop/range' },
            { text: 'generate', link: '/fr/guide/creation-functions/loop/generate' },
          ],
        },
        {
          text: 'Communication HTTP',
          link: '/fr/guide/creation-functions/http-communication/',
          collapsed: true,
          items: [
            { text: 'ajax', link: '/fr/guide/creation-functions/http-communication/ajax' },
            { text: 'fromFetch', link: '/fr/guide/creation-functions/http-communication/fromFetch' },
          ],
        },
        {
          text: 'Combinaison',
          link: '/fr/guide/creation-functions/combination/',
          collapsed: true,
          items: [
            { text: 'concat', link: '/fr/guide/creation-functions/combination/concat' },
            { text: 'merge', link: '/fr/guide/creation-functions/combination/merge' },
            { text: 'combineLatest', link: '/fr/guide/creation-functions/combination/combineLatest' },
            { text: 'zip', link: '/fr/guide/creation-functions/combination/zip' },
            { text: 'forkJoin', link: '/fr/guide/creation-functions/combination/forkJoin' },
          ],
        },
        {
          text: 'Sélection/Partition',
          link: '/fr/guide/creation-functions/selection/',
          collapsed: true,
          items: [
            { text: 'race', link: '/fr/guide/creation-functions/selection/race' },
            { text: 'partition', link: '/fr/guide/creation-functions/selection/partition' },
          ],
        },
        {
          text: 'Branchement conditionnel',
          link: '/fr/guide/creation-functions/conditional/',
          collapsed: true,
          items: [
            { text: 'iif', link: '/fr/guide/creation-functions/conditional/iif' },
            { text: 'defer', link: '/fr/guide/creation-functions/conditional/defer' },
          ],
        },
        {
          text: 'Contrôle',
          link: '/fr/guide/creation-functions/control/',
          collapsed: true,
          items: [
            { text: 'scheduled', link: '/fr/guide/creation-functions/control/scheduled' },
            { text: 'using', link: '/fr/guide/creation-functions/control/using' },
          ],
        },
      ],
    },
    {
      text: '4. Comprendre les opérateurs',
      link: '/fr/guide/operators/',
      items: [
        { text: 'Concepts de pipeline', link: '/fr/guide/operators/pipeline' },
        {
          text: 'Opérateurs de transformation',
          link: '/fr/guide/operators/transformation/',
          collapsed: true,
          items: [
            { text: 'map', link: '/fr/guide/operators/transformation/map' },
            { text: 'scan', link: '/fr/guide/operators/transformation/scan' },
            { text: 'mergeMap', link: '/fr/guide/operators/transformation/mergeMap' },
            { text: 'switchMap', link: '/fr/guide/operators/transformation/switchMap' },
            { text: 'concatMap', link: '/fr/guide/operators/transformation/concatMap' },
            { text: 'exhaustMap', link: '/fr/guide/operators/transformation/exhaustMap' },
            { text: 'Cas pratiques', link: '/fr/guide/operators/transformation/practical-use-cases' },
          ],
        },
        {
          text: 'Opérateurs de filtrage',
          link: '/fr/guide/operators/filtering/',
          collapsed: true,
          items: [
            { text: 'filter', link: '/fr/guide/operators/filtering/filter' },
            { text: 'take', link: '/fr/guide/operators/filtering/take' },
            { text: 'debounceTime', link: '/fr/guide/operators/filtering/debounceTime' },
            { text: 'throttleTime', link: '/fr/guide/operators/filtering/throttleTime' },
            { text: 'distinctUntilChanged', link: '/fr/guide/operators/filtering/distinctUntilChanged' },
            { text: 'Cas pratiques', link: '/fr/guide/operators/filtering/practical-use-cases' },
          ],
        },
        {
          text: 'Opérateurs de combinaison',
          link: '/fr/guide/operators/combination/',
          collapsed: true,
          items: [
            { text: 'withLatestFrom', link: '/fr/guide/operators/combination/withLatestFrom' },
            { text: 'mergeWith', link: '/fr/guide/operators/combination/mergeWith' },
            { text: 'Cas pratiques', link: '/fr/guide/operators/combination/practical-use-cases' },
          ],
        },
        {
          text: 'Opérateurs utilitaires',
          link: '/fr/guide/operators/utility/',
          collapsed: true,
          items: [
            { text: 'tap', link: '/fr/guide/operators/utility/tap' },
            { text: 'delay', link: '/fr/guide/operators/utility/delay' },
            { text: 'takeUntil', link: '/fr/guide/operators/utility/takeUntil' },
            { text: 'finalize', link: '/fr/guide/operators/utility/finalize' },
            { text: 'retry', link: '/fr/guide/operators/utility/retry' },
            { text: 'Cas pratiques', link: '/fr/guide/operators/utility/practical-use-cases' },
          ],
        },
        {
          text: 'Multicasting',
          link: '/fr/guide/operators/multicasting/',
          collapsed: true,
          items: [
            { text: 'share', link: '/fr/guide/operators/multicasting/share' },
            { text: 'shareReplay', link: '/fr/guide/operators/multicasting/shareReplay' },
          ],
        },
      ],
    },
    {
      text: '5. Subject et Multicast',
      items: [
        { text: "Qu'est-ce qu'un Subject ?", link: '/fr/guide/subjects/what-is-subject' },
        { text: 'Types de Subject', link: '/fr/guide/subjects/types-of-subject' },
        { text: 'Fonctionnement du Multicasting', link: '/fr/guide/subjects/multicasting' },
        { text: "Cas d'utilisation des Subject", link: '/fr/guide/subjects/use-cases' },
      ],
    },
    {
      text: '6. Gestion des erreurs',
      items: [
        { text: 'Stratégies de gestion des erreurs', link: '/fr/guide/error-handling/strategies' },
        { text: 'Deux emplacements pour la gestion des erreurs', link: '/fr/guide/error-handling/error-handling-locations' },
        { text: 'Intégration try-catch et RxJS', link: '/fr/guide/error-handling/try-catch-integration' },
        { text: 'retry et catchError', link: '/fr/guide/error-handling/retry-catch' },
        { text: 'finalize et complete', link: '/fr/guide/error-handling/finalize' },
      ],
    },
    {
      text: '7. Utilisation des Schedulers',
      items: [
        { text: 'Contrôle du traitement asynchrone', link: '/fr/guide/schedulers/async-control' },
        { text: 'Types de Schedulers', link: '/fr/guide/schedulers/types' },
        { text: 'Bases des tâches et Schedulers', link: '/fr/guide/schedulers/task-and-scheduler-basics' },
      ],
    },
    {
      text: '8. Techniques de débogage RxJS',
      items: [
        { text: 'Aperçu des techniques de débogage', link: '/fr/guide/debugging/' },
        { text: 'Scénarios de débogage courants', link: '/fr/guide/debugging/common-scenarios' },
        { text: 'Outils de débogage personnalisés', link: '/fr/guide/debugging/custom-tools' },
        { text: 'Débogage des performances', link: '/fr/guide/debugging/performance' },
      ],
    },
    {
      text: '9. Méthodes de test',
      items: [
        { text: 'Tests unitaires RxJS', link: '/fr/guide/testing/unit-tests' },
        { text: 'Utilisation de TestScheduler', link: '/fr/guide/testing/test-scheduler' },
        { text: 'Test Marble', link: '/fr/guide/testing/marble-testing' },
      ],
    },
    {
      text: '10. Anti-patterns RxJS',
      items: [
        { text: "Qu'est-ce qu'un anti-pattern ?", link: '/fr/guide/anti-patterns/' },
        { text: 'Erreurs courantes et solutions', link: '/fr/guide/anti-patterns/common-mistakes' },
        { text: 'Prolifération des flags', link: '/fr/guide/anti-patterns/flag-management' },
        { text: 'Enfer des if dans subscribe', link: '/fr/guide/anti-patterns/subscribe-if-hell' },
        { text: 'Mélange Promise et Observable', link: '/fr/guide/anti-patterns/promise-observable-mixing' },
        { text: 'Enfer des one-liners', link: '/fr/guide/anti-patterns/one-liner-hell' },
        { text: 'Checklist anti-patterns', link: '/fr/guide/anti-patterns/checklist' },
      ],
    },
    {
      text: '11. Surmonter les difficultés RxJS',
      items: [
        { text: 'Pourquoi RxJS est-il difficile ?', link: '/fr/guide/overcoming-difficulties/' },
        { text: 'Compréhension conceptuelle', link: '/fr/guide/overcoming-difficulties/conceptual-understanding' },
        { text: 'Gestion du cycle de vie', link: '/fr/guide/overcoming-difficulties/lifecycle-management' },
        { text: 'Choix des opérateurs', link: '/fr/guide/overcoming-difficulties/operator-selection' },
        { text: 'Timing et ordre', link: '/fr/guide/overcoming-difficulties/timing-and-order' },
        { text: 'Gestion de l\'état', link: '/fr/guide/overcoming-difficulties/state-and-sharing' },
        { text: 'Combinaison de streams', link: '/fr/guide/overcoming-difficulties/stream-combination' },
        { text: 'Guide de débogage', link: '/fr/guide/overcoming-difficulties/debugging-guide' },
      ],
    },
    {
      text: '13. Patterns pratiques',
      items: [
        { text: 'Aperçu des patterns', link: '/fr/guide/practical-patterns/' },
        { text: 'Traitement des événements UI', link: '/fr/guide/practical-patterns/ui-events' },
        { text: 'Appels API', link: '/fr/guide/practical-patterns/api-calls' },
        { text: 'Traitement des formulaires', link: '/fr/guide/practical-patterns/form-handling' },
        { text: 'Patterns de formulaires avancés', link: '/fr/guide/practical-patterns/advanced-form-patterns' },
        { text: 'Données en temps réel', link: '/fr/guide/practical-patterns/real-time-data' },
        { text: 'Stratégies de cache', link: '/fr/guide/practical-patterns/caching-strategies' },
        { text: 'Patterns de gestion des erreurs', link: '/fr/guide/practical-patterns/error-handling-patterns' },
        { text: 'Branchement dans subscribe', link: '/fr/guide/practical-patterns/subscribe-branching' },
      ],
    },
    {
      text: 'Annexe',
      items: [
        { text: 'Aperçu de l\'annexe', link: '/fr/guide/appendix/' },
        { text: 'Développement embarqué et RP', link: '/fr/guide/appendix/embedded-reactive-programming' },
        { text: 'Méthodes réactives hors ReactiveX', link: '/fr/guide/appendix/reactive-patterns-beyond-rxjs' },
        { text: 'Carte d\'architecture réactive', link: '/fr/guide/appendix/reactive-architecture-map' },
        { text: 'Programmation réactive reconsidérée', link: '/fr/guide/appendix/reactive-programming-reconsidered' },
        { text: 'Écosystème RxJS et Reactive Streams', link: '/fr/guide/appendix/rxjs-and-reactive-streams-ecosystem' },
      ],
    },
  ],

  socialLinks: [
    {
      icon: 'github',
      link: 'https://github.com/shuji-bonji/RxJS-with-TypeScript',
    },
  ],

  footer: {
    message: 'Publié sous licence CC-BY-4.0.',
    copyright: 'Copyright © 2025 shuji-bonji',
  },
};
