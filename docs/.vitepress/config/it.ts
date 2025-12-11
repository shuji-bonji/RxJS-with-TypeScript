import { DefaultTheme } from 'vitepress';

export const itThemeConfig: DefaultTheme.Config = {
  nav: [
    { text: 'Home', link: '/it/' },
    { text: 'Guida', link: '/it/guide/' },
  ],
  sidebar: [
    {
      text: '1. Introduzione a RxJS',
      items: [
        { text: 'Introduzione', link: '/it/guide/introduction' },
        { text: "Configurazione dell'ambiente", link: '/it/guide/starter-kid' },
        { text: "Cos'è RxJS?", link: '/it/guide/basics/what-is-rxjs' },
        { text: "Cos'è uno Stream?", link: '/it/guide/basics/what-is-a-stream' },
        { text: 'Differenza tra Promise e RxJS', link: '/it/guide/basics/promise-vs-rxjs' },
      ],
    },
    {
      text: '2. Fondamenti di Observable',
      items: [
        { text: "Cos'è un Observable?", link: '/it/guide/observables/what-is-observable' },
        { text: 'Creare un Observable', link: '/it/guide/observables/creation' },
        { text: 'Streamificare gli eventi', link: '/it/guide/observables/events' },
        { text: 'Lista degli eventi', link: '/it/guide/observables/events-list' },
        { text: 'Observer vs Subscriber', link: '/it/guide/observables/observer-vs-subscriber' },
        { text: 'Ciclo di vita Observable', link: '/it/guide/observables/observable-lifecycle' },
        { text: 'Cold e Hot Observable', link: '/it/guide/observables/cold-and-hot-observables' },
      ],
    },
    {
      text: '3. Funzioni di creazione',
      link: '/it/guide/creation-functions/',
      items: [
        {
          text: 'Creazione base',
          link: '/it/guide/creation-functions/basic/',
          collapsed: true,
          items: [
            { text: 'of', link: '/it/guide/creation-functions/basic/of' },
            { text: 'from', link: '/it/guide/creation-functions/basic/from' },
            { text: 'fromEvent', link: '/it/guide/creation-functions/basic/fromEvent' },
            { text: 'interval', link: '/it/guide/creation-functions/basic/interval' },
            { text: 'timer', link: '/it/guide/creation-functions/basic/timer' },
          ],
        },
        {
          text: 'Generazione ciclica',
          link: '/it/guide/creation-functions/loop/',
          collapsed: true,
          items: [
            { text: 'range', link: '/it/guide/creation-functions/loop/range' },
            { text: 'generate', link: '/it/guide/creation-functions/loop/generate' },
          ],
        },
        {
          text: 'Comunicazione HTTP',
          link: '/it/guide/creation-functions/http-communication/',
          collapsed: true,
          items: [
            { text: 'ajax', link: '/it/guide/creation-functions/http-communication/ajax' },
            { text: 'fromFetch', link: '/it/guide/creation-functions/http-communication/fromFetch' },
          ],
        },
        {
          text: 'Combinazione',
          link: '/it/guide/creation-functions/combination/',
          collapsed: true,
          items: [
            { text: 'concat', link: '/it/guide/creation-functions/combination/concat' },
            { text: 'merge', link: '/it/guide/creation-functions/combination/merge' },
            { text: 'combineLatest', link: '/it/guide/creation-functions/combination/combineLatest' },
            { text: 'zip', link: '/it/guide/creation-functions/combination/zip' },
            { text: 'forkJoin', link: '/it/guide/creation-functions/combination/forkJoin' },
            { text: 'forkJoin vs combineLatest', link: '/it/guide/creation-functions/combination/forkJoin-vs-combineLatest' },
          ],
        },
        {
          text: 'Selezione/Partizione',
          link: '/it/guide/creation-functions/selection/',
          collapsed: true,
          items: [
            { text: 'race', link: '/it/guide/creation-functions/selection/race' },
            { text: 'partition', link: '/it/guide/creation-functions/selection/partition' },
          ],
        },
        {
          text: 'Diramazione condizionale',
          link: '/it/guide/creation-functions/conditional/',
          collapsed: true,
          items: [
            { text: 'iif', link: '/it/guide/creation-functions/conditional/iif' },
            { text: 'defer', link: '/it/guide/creation-functions/conditional/defer' },
          ],
        },
        {
          text: 'Controllo',
          link: '/it/guide/creation-functions/control/',
          collapsed: true,
          items: [
            { text: 'scheduled', link: '/it/guide/creation-functions/control/scheduled' },
            { text: 'using', link: '/it/guide/creation-functions/control/using' },
          ],
        },
      ],
    },
    {
      text: '4. Comprendere gli operatori',
      link: '/it/guide/operators/',
      items: [
        { text: 'Concetti di pipeline', link: '/it/guide/operators/pipeline' },
        {
          text: 'Operatori di trasformazione',
          link: '/it/guide/operators/transformation/',
          collapsed: true,
          items: [
            { text: 'map', link: '/it/guide/operators/transformation/map' },
            { text: 'scan', link: '/it/guide/operators/transformation/scan' },
            { text: 'mergeMap', link: '/it/guide/operators/transformation/mergeMap' },
            { text: 'switchMap', link: '/it/guide/operators/transformation/switchMap' },
            { text: 'concatMap', link: '/it/guide/operators/transformation/concatMap' },
            { text: 'exhaustMap', link: '/it/guide/operators/transformation/exhaustMap' },
            { text: "Casi d'uso pratici", link: '/it/guide/operators/transformation/practical-use-cases' },
          ],
        },
        {
          text: 'Operatori di filtro',
          link: '/it/guide/operators/filtering/',
          collapsed: true,
          items: [
            { text: 'filter', link: '/it/guide/operators/filtering/filter' },
            { text: 'take', link: '/it/guide/operators/filtering/take' },
            { text: 'debounceTime', link: '/it/guide/operators/filtering/debounceTime' },
            { text: 'throttleTime', link: '/it/guide/operators/filtering/throttleTime' },
            { text: 'distinctUntilChanged', link: '/it/guide/operators/filtering/distinctUntilChanged' },
            { text: "Casi d'uso pratici", link: '/it/guide/operators/filtering/practical-use-cases' },
          ],
        },
        {
          text: 'Operatori di combinazione',
          link: '/it/guide/operators/combination/',
          collapsed: true,
          items: [
            { text: 'withLatestFrom', link: '/it/guide/operators/combination/withLatestFrom' },
            { text: 'mergeWith', link: '/it/guide/operators/combination/mergeWith' },
            { text: "Casi d'uso pratici", link: '/it/guide/operators/combination/practical-use-cases' },
          ],
        },
        {
          text: 'Operatori di utilità',
          link: '/it/guide/operators/utility/',
          collapsed: true,
          items: [
            { text: 'tap', link: '/it/guide/operators/utility/tap' },
            { text: 'delay', link: '/it/guide/operators/utility/delay' },
            { text: 'takeUntil', link: '/it/guide/operators/utility/takeUntil' },
            { text: 'finalize', link: '/it/guide/operators/utility/finalize' },
            { text: 'retry', link: '/it/guide/operators/utility/retry' },
            { text: "Casi d'uso pratici", link: '/it/guide/operators/utility/practical-use-cases' },
          ],
        },
        {
          text: 'Multicasting',
          link: '/it/guide/operators/multicasting/',
          collapsed: true,
          items: [
            { text: 'share', link: '/it/guide/operators/multicasting/share' },
            { text: 'shareReplay', link: '/it/guide/operators/multicasting/shareReplay' },
          ],
        },
      ],
    },
    {
      text: '5. Subject e Multicast',
      items: [
        { text: "Cos'è un Subject?", link: '/it/guide/subjects/what-is-subject' },
        { text: 'Tipi di Subject', link: '/it/guide/subjects/types-of-subject' },
        { text: 'Meccanismo di Multicasting', link: '/it/guide/subjects/multicasting' },
        { text: "Casi d'uso di Subject", link: '/it/guide/subjects/use-cases' },
      ],
    },
    {
      text: '6. Gestione degli errori',
      items: [
        { text: 'Strategie di gestione errori', link: '/it/guide/error-handling/strategies' },
        { text: 'Due posizioni per la gestione errori', link: '/it/guide/error-handling/error-handling-locations' },
        { text: 'Integrazione try-catch e RxJS', link: '/it/guide/error-handling/try-catch-integration' },
        { text: 'retry e catchError', link: '/it/guide/error-handling/retry-catch' },
        { text: 'finalize e complete', link: '/it/guide/error-handling/finalize' },
      ],
    },
    {
      text: '7. Utilizzo degli Scheduler',
      items: [
        { text: 'Controllo elaborazione asincrona', link: '/it/guide/schedulers/async-control' },
        { text: 'Tipi di Scheduler', link: '/it/guide/schedulers/types' },
        { text: 'Basi di task e Scheduler', link: '/it/guide/schedulers/task-and-scheduler-basics' },
      ],
    },
    {
      text: '8. Tecniche di debug RxJS',
      items: [
        { text: 'Panoramica tecniche di debug', link: '/it/guide/debugging/' },
        { text: 'Scenari di debug comuni', link: '/it/guide/debugging/common-scenarios' },
        { text: 'Strumenti di debug personalizzati', link: '/it/guide/debugging/custom-tools' },
        { text: 'Debug delle prestazioni', link: '/it/guide/debugging/performance' },
      ],
    },
    {
      text: '9. Metodi di test',
      items: [
        { text: 'Test unitari RxJS', link: '/it/guide/testing/unit-tests' },
        { text: 'Utilizzo di TestScheduler', link: '/it/guide/testing/test-scheduler' },
        { text: 'Marble Testing', link: '/it/guide/testing/marble-testing' },
      ],
    },
    {
      text: '10. Anti-pattern RxJS',
      items: [
        { text: "Cos'è un anti-pattern?", link: '/it/guide/anti-patterns/' },
        { text: 'Errori comuni e soluzioni', link: '/it/guide/anti-patterns/common-mistakes' },
        { text: 'Proliferazione dei flag', link: '/it/guide/anti-patterns/flag-management' },
        { text: 'Inferno degli if in subscribe', link: '/it/guide/anti-patterns/subscribe-if-hell' },
        { text: 'Mescolare Promise e Observable', link: '/it/guide/anti-patterns/promise-observable-mixing' },
        { text: 'Inferno delle one-liner', link: '/it/guide/anti-patterns/one-liner-hell' },
        { text: 'Checklist anti-pattern', link: '/it/guide/anti-patterns/checklist' },
      ],
    },
    {
      text: '11. Superare le difficoltà RxJS',
      items: [
        { text: 'Perché RxJS è difficile?', link: '/it/guide/overcoming-difficulties/' },
        { text: 'Comprensione concettuale', link: '/it/guide/overcoming-difficulties/conceptual-understanding' },
        { text: 'Gestione del ciclo di vita', link: '/it/guide/overcoming-difficulties/lifecycle-management' },
        { text: 'Scelta degli operatori', link: '/it/guide/overcoming-difficulties/operator-selection' },
        { text: 'Timing e ordine', link: '/it/guide/overcoming-difficulties/timing-and-order' },
        { text: 'Gestione dello stato', link: '/it/guide/overcoming-difficulties/state-and-sharing' },
        { text: 'Combinazione di stream', link: '/it/guide/overcoming-difficulties/stream-combination' },
        { text: 'Guida al debug', link: '/it/guide/overcoming-difficulties/debugging-guide' },
      ],
    },
    {
      text: '13. Pattern pratici',
      items: [
        { text: 'Panoramica pattern', link: '/it/guide/practical-patterns/' },
        { text: 'Elaborazione eventi UI', link: '/it/guide/practical-patterns/ui-events' },
        { text: 'Chiamate API', link: '/it/guide/practical-patterns/api-calls' },
        { text: 'Elaborazione form', link: '/it/guide/practical-patterns/form-handling' },
        { text: 'Pattern form avanzati', link: '/it/guide/practical-patterns/advanced-form-patterns' },
        { text: 'Dati in tempo reale', link: '/it/guide/practical-patterns/real-time-data' },
        { text: 'Strategie di caching', link: '/it/guide/practical-patterns/caching-strategies' },
        { text: 'Pattern gestione errori', link: '/it/guide/practical-patterns/error-handling-patterns' },
        { text: 'Diramazione in subscribe', link: '/it/guide/practical-patterns/subscribe-branching' },
      ],
    },
    {
      text: 'Appendice',
      items: [
        { text: "Panoramica dell'appendice", link: '/it/guide/appendix/' },
        { text: 'Sviluppo embedded e RP', link: '/it/guide/appendix/embedded-reactive-programming' },
        { text: 'Metodi reattivi oltre ReactiveX', link: '/it/guide/appendix/reactive-patterns-beyond-rxjs' },
        { text: 'Mappa architettura reattiva', link: '/it/guide/appendix/reactive-architecture-map' },
        { text: 'Programmazione reattiva riconsiderata', link: '/it/guide/appendix/reactive-programming-reconsidered' },
        { text: 'Ecosistema RxJS e Reactive Streams', link: '/it/guide/appendix/rxjs-and-reactive-streams-ecosystem' },
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
    message: 'Pubblicato sotto licenza CC-BY-4.0.',
    copyright: 'Copyright © 2025 shuji-bonji',
  },
};
