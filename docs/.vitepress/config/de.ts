import { DefaultTheme } from 'vitepress';

export const deThemeConfig: DefaultTheme.Config = {
  nav: [
    { text: 'Startseite', link: '/de/' },
    { text: 'Anleitung', link: '/de/guide/' },
  ],
  sidebar: [
    {
      text: '1. Einführung in RxJS',
      items: [
        { text: 'Einführung', link: '/de/guide/introduction' },
        { text: 'Entwicklungsumgebung einrichten', link: '/de/guide/starter-kid' },
        { text: 'Was ist RxJS?', link: '/de/guide/basics/what-is-rxjs' },
        { text: 'Was ist ein Stream?', link: '/de/guide/basics/what-is-a-stream' },
        { text: 'Unterschied zwischen Promise und RxJS', link: '/de/guide/basics/promise-vs-rxjs' },
      ],
    },
    {
      text: '2. Observable-Grundlagen',
      items: [
        { text: 'Was ist ein Observable?', link: '/de/guide/observables/what-is-observable' },
        { text: 'Observable erstellen', link: '/de/guide/observables/creation' },
        { text: 'Ereignisse streamen', link: '/de/guide/observables/events' },
        { text: 'Ereignisliste', link: '/de/guide/observables/events-list' },
        { text: 'Observer vs Subscriber', link: '/de/guide/observables/observer-vs-subscriber' },
        { text: 'Observable-Lebenszyklus', link: '/de/guide/observables/observable-lifecycle' },
        { text: 'Cold und Hot Observable', link: '/de/guide/observables/cold-and-hot-observables' },
      ],
    },
    {
      text: '3. Erstellungsfunktionen',
      link: '/de/guide/creation-functions/',
      items: [
        {
          text: 'Grundlegende Erstellung',
          link: '/de/guide/creation-functions/basic/',
          collapsed: true,
          items: [
            { text: 'of', link: '/de/guide/creation-functions/basic/of' },
            { text: 'from', link: '/de/guide/creation-functions/basic/from' },
            { text: 'fromEvent', link: '/de/guide/creation-functions/basic/fromEvent' },
            { text: 'interval', link: '/de/guide/creation-functions/basic/interval' },
            { text: 'timer', link: '/de/guide/creation-functions/basic/timer' },
          ],
        },
        {
          text: 'Schleifengenerierung',
          link: '/de/guide/creation-functions/loop/',
          collapsed: true,
          items: [
            { text: 'range', link: '/de/guide/creation-functions/loop/range' },
            { text: 'generate', link: '/de/guide/creation-functions/loop/generate' },
          ],
        },
        {
          text: 'HTTP-Kommunikation',
          link: '/de/guide/creation-functions/http-communication/',
          collapsed: true,
          items: [
            { text: 'ajax', link: '/de/guide/creation-functions/http-communication/ajax' },
            { text: 'fromFetch', link: '/de/guide/creation-functions/http-communication/fromFetch' },
          ],
        },
        {
          text: 'Kombination',
          link: '/de/guide/creation-functions/combination/',
          collapsed: true,
          items: [
            { text: 'concat', link: '/de/guide/creation-functions/combination/concat' },
            { text: 'merge', link: '/de/guide/creation-functions/combination/merge' },
            { text: 'combineLatest', link: '/de/guide/creation-functions/combination/combineLatest' },
            { text: 'zip', link: '/de/guide/creation-functions/combination/zip' },
            { text: 'forkJoin', link: '/de/guide/creation-functions/combination/forkJoin' },
          ],
        },
        {
          text: 'Auswahl/Partitionierung',
          link: '/de/guide/creation-functions/selection/',
          collapsed: true,
          items: [
            { text: 'race', link: '/de/guide/creation-functions/selection/race' },
            { text: 'partition', link: '/de/guide/creation-functions/selection/partition' },
          ],
        },
        {
          text: 'Bedingte Verzweigung',
          link: '/de/guide/creation-functions/conditional/',
          collapsed: true,
          items: [
            { text: 'iif', link: '/de/guide/creation-functions/conditional/iif' },
            { text: 'defer', link: '/de/guide/creation-functions/conditional/defer' },
          ],
        },
        {
          text: 'Steuerung',
          link: '/de/guide/creation-functions/control/',
          collapsed: true,
          items: [
            { text: 'scheduled', link: '/de/guide/creation-functions/control/scheduled' },
            { text: 'using', link: '/de/guide/creation-functions/control/using' },
          ],
        },
      ],
    },
    {
      text: '4. Operatoren verstehen',
      link: '/de/guide/operators/',
      items: [
        { text: 'Pipeline-Konzepte', link: '/de/guide/operators/pipeline' },
        {
          text: 'Transformationsoperatoren',
          link: '/de/guide/operators/transformation/',
          collapsed: true,
          items: [
            { text: 'map', link: '/de/guide/operators/transformation/map' },
            { text: 'scan', link: '/de/guide/operators/transformation/scan' },
            { text: 'mergeMap', link: '/de/guide/operators/transformation/mergeMap' },
            { text: 'switchMap', link: '/de/guide/operators/transformation/switchMap' },
            { text: 'concatMap', link: '/de/guide/operators/transformation/concatMap' },
            { text: 'exhaustMap', link: '/de/guide/operators/transformation/exhaustMap' },
            { text: 'Praktische Anwendungsfälle', link: '/de/guide/operators/transformation/practical-use-cases' },
          ],
        },
        {
          text: 'Filteroperatoren',
          link: '/de/guide/operators/filtering/',
          collapsed: true,
          items: [
            { text: 'filter', link: '/de/guide/operators/filtering/filter' },
            { text: 'take', link: '/de/guide/operators/filtering/take' },
            { text: 'debounceTime', link: '/de/guide/operators/filtering/debounceTime' },
            { text: 'throttleTime', link: '/de/guide/operators/filtering/throttleTime' },
            { text: 'distinctUntilChanged', link: '/de/guide/operators/filtering/distinctUntilChanged' },
            { text: 'Praktische Anwendungsfälle', link: '/de/guide/operators/filtering/practical-use-cases' },
          ],
        },
        {
          text: 'Kombinationsoperatoren',
          link: '/de/guide/operators/combination/',
          collapsed: true,
          items: [
            { text: 'withLatestFrom', link: '/de/guide/operators/combination/withLatestFrom' },
            { text: 'mergeWith', link: '/de/guide/operators/combination/mergeWith' },
            { text: 'Praktische Anwendungsfälle', link: '/de/guide/operators/combination/practical-use-cases' },
          ],
        },
        {
          text: 'Hilfsoperatoren',
          link: '/de/guide/operators/utility/',
          collapsed: true,
          items: [
            { text: 'tap', link: '/de/guide/operators/utility/tap' },
            { text: 'delay', link: '/de/guide/operators/utility/delay' },
            { text: 'takeUntil', link: '/de/guide/operators/utility/takeUntil' },
            { text: 'finalize', link: '/de/guide/operators/utility/finalize' },
            { text: 'retry', link: '/de/guide/operators/utility/retry' },
            { text: 'Praktische Anwendungsfälle', link: '/de/guide/operators/utility/practical-use-cases' },
          ],
        },
        {
          text: 'Multicasting',
          link: '/de/guide/operators/multicasting/',
          collapsed: true,
          items: [
            { text: 'share', link: '/de/guide/operators/multicasting/share' },
            { text: 'shareReplay', link: '/de/guide/operators/multicasting/shareReplay' },
          ],
        },
      ],
    },
    {
      text: '5. Subject und Multicast',
      items: [
        { text: 'Was ist ein Subject?', link: '/de/guide/subjects/what-is-subject' },
        { text: 'Subject-Typen', link: '/de/guide/subjects/types-of-subject' },
        { text: 'Multicasting-Mechanismus', link: '/de/guide/subjects/multicasting' },
        { text: 'Subject-Anwendungsfälle', link: '/de/guide/subjects/use-cases' },
      ],
    },
    {
      text: '6. Fehlerbehandlung',
      items: [
        { text: 'Fehlerbehandlungsstrategien', link: '/de/guide/error-handling/strategies' },
        { text: 'Zwei Orte für Fehlerbehandlung', link: '/de/guide/error-handling/error-handling-locations' },
        { text: 'try-catch und RxJS-Integration', link: '/de/guide/error-handling/try-catch-integration' },
        { text: 'retry und catchError', link: '/de/guide/error-handling/retry-catch' },
        { text: 'finalize und complete', link: '/de/guide/error-handling/finalize' },
      ],
    },
    {
      text: '7. Scheduler-Nutzung',
      items: [
        { text: 'Asynchrone Verarbeitungssteuerung', link: '/de/guide/schedulers/async-control' },
        { text: 'Scheduler-Typen', link: '/de/guide/schedulers/types' },
        { text: 'Aufgaben- und Scheduler-Grundlagen', link: '/de/guide/schedulers/task-and-scheduler-basics' },
      ],
    },
    {
      text: '8. RxJS-Debugging-Techniken',
      items: [
        { text: 'Überblick Debugging-Techniken', link: '/de/guide/debugging/' },
        { text: 'Häufige Debugging-Szenarien', link: '/de/guide/debugging/common-scenarios' },
        { text: 'Benutzerdefinierte Debugging-Tools', link: '/de/guide/debugging/custom-tools' },
        { text: 'Performance-Debugging', link: '/de/guide/debugging/performance' },
      ],
    },
    {
      text: '9. Testmethoden',
      items: [
        { text: 'RxJS-Unit-Tests', link: '/de/guide/testing/unit-tests' },
        { text: 'TestScheduler-Nutzung', link: '/de/guide/testing/test-scheduler' },
        { text: 'Marble-Testing', link: '/de/guide/testing/marble-testing' },
      ],
    },
    {
      text: '10. RxJS-Anti-Patterns',
      items: [
        { text: 'Was ist ein Anti-Pattern?', link: '/de/guide/anti-patterns/' },
        { text: 'Häufige Fehler und Lösungen', link: '/de/guide/anti-patterns/common-mistakes' },
        { text: 'Flag-Wildwuchs', link: '/de/guide/anti-patterns/flag-management' },
        { text: 'If-Hölle in subscribe', link: '/de/guide/anti-patterns/subscribe-if-hell' },
        { text: 'Promise und Observable mischen', link: '/de/guide/anti-patterns/promise-observable-mixing' },
        { text: 'Einzeiler-Hölle', link: '/de/guide/anti-patterns/one-liner-hell' },
        { text: 'Anti-Pattern-Checkliste', link: '/de/guide/anti-patterns/checklist' },
      ],
    },
    {
      text: '11. RxJS-Schwierigkeiten überwinden',
      items: [
        { text: 'Warum ist RxJS schwierig?', link: '/de/guide/overcoming-difficulties/' },
        { text: 'Konzeptionelles Verständnis', link: '/de/guide/overcoming-difficulties/conceptual-understanding' },
        { text: 'Lebenszyklusverwaltung', link: '/de/guide/overcoming-difficulties/lifecycle-management' },
        { text: 'Operatorauswahl', link: '/de/guide/overcoming-difficulties/operator-selection' },
        { text: 'Timing und Reihenfolge', link: '/de/guide/overcoming-difficulties/timing-and-order' },
        { text: 'Zustandsverwaltung', link: '/de/guide/overcoming-difficulties/state-and-sharing' },
        { text: 'Stream-Kombination', link: '/de/guide/overcoming-difficulties/stream-combination' },
        { text: 'Debugging-Anleitung', link: '/de/guide/overcoming-difficulties/debugging-guide' },
      ],
    },
    {
      text: '13. Praktische Muster',
      items: [
        { text: 'Musterübersicht', link: '/de/guide/practical-patterns/' },
        { text: 'UI-Ereignisverarbeitung', link: '/de/guide/practical-patterns/ui-events' },
        { text: 'API-Aufrufe', link: '/de/guide/practical-patterns/api-calls' },
        { text: 'Formularverarbeitung', link: '/de/guide/practical-patterns/form-handling' },
        { text: 'Erweiterte Formularmuster', link: '/de/guide/practical-patterns/advanced-form-patterns' },
        { text: 'Echtzeitdaten', link: '/de/guide/practical-patterns/real-time-data' },
        { text: 'Caching-Strategien', link: '/de/guide/practical-patterns/caching-strategies' },
        { text: 'Fehlerbehandlungsmuster', link: '/de/guide/practical-patterns/error-handling-patterns' },
        { text: 'Verzweigung in subscribe', link: '/de/guide/practical-patterns/subscribe-branching' },
      ],
    },
    {
      text: 'Anhang',
      items: [
        { text: 'Anhangsübersicht', link: '/de/guide/appendix/' },
        { text: 'Eingebettete Entwicklung und RP', link: '/de/guide/appendix/embedded-reactive-programming' },
        { text: 'Reaktive Methoden außerhalb ReactiveX', link: '/de/guide/appendix/reactive-patterns-beyond-rxjs' },
        { text: 'Reaktive Architektur-Karte', link: '/de/guide/appendix/reactive-architecture-map' },
        { text: 'Reaktive Programmierung überdacht', link: '/de/guide/appendix/reactive-programming-reconsidered' },
        { text: 'RxJS und Reactive Streams-Ökosystem', link: '/de/guide/appendix/rxjs-and-reactive-streams-ecosystem' },
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
    message: 'Veröffentlicht unter CC-BY-4.0-Lizenz.',
    copyright: 'Copyright © 2025 shuji-bonji',
  },
};
