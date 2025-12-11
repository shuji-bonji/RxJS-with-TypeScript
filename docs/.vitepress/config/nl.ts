import { DefaultTheme } from 'vitepress';

export const nlThemeConfig: DefaultTheme.Config = {
  nav: [
    { text: 'Home', link: '/nl/' },
    { text: 'Gids', link: '/nl/guide/' },
  ],
  sidebar: [
    {
      text: '1. Introductie tot RxJS',
      items: [
        { text: 'Introductie', link: '/nl/guide/introduction' },
        { text: 'Praktische Leeromgeving Opzetten', link: '/nl/guide/starter-kid.md' },
        { text: 'Wat is RxJS?', link: '/nl/guide/basics/what-is-rxjs' },
        { text: 'Wat is een Stream?', link: '/nl/guide/basics/what-is-a-stream' },
        { text: 'Verschil tussen Promise en RxJS', link: '/nl/guide/basics/promise-vs-rxjs' },
      ],
    },
    {
      text: '2. Basis van Observable',
      items: [
        {
          text: 'Wat is een Observable?',
          link: '/nl/guide/observables/what-is-observable',
        },
        { text: 'Een Observable Maken', link: '/nl/guide/observables/creation' },
        { text: 'Gebeurtenissen Streamen', link: '/nl/guide/observables/events' },
        {
          text: 'Niet beschikbaar in fromEvent',
          link: '/nl/guide/observables/events#cannot-used-fromEvent'
        },
        { text: 'Lijst van Gebeurtenissen', link: '/nl/guide/observables/events-list' },
        {
          text: 'Verschil tussen Observer en Subscriber',
          link: '/nl/guide/observables/observer-vs-subscriber',
        },
        {
          text: 'Observable Levenscyclus',
          link: '/nl/guide/observables/observable-lifecycle',
        },
        {
          text: 'Cold Observable en Hot Observable',
          link: '/nl/guide/observables/cold-and-hot-observables',
        },
      ],
    },
    {
      text: '3. Creation Functions',
      link: '/nl/guide/creation-functions/',
      items: [
        {
          text: 'Basis Creatie',
          link: '/nl/guide/creation-functions/basic/',
          collapsed: true,
          items: [
            { text: 'of', link: '/nl/guide/creation-functions/basic/of' },
            { text: 'from', link: '/nl/guide/creation-functions/basic/from' },
            { text: 'fromEvent', link: '/nl/guide/creation-functions/basic/fromEvent' },
            { text: 'interval', link: '/nl/guide/creation-functions/basic/interval' },
            { text: 'timer', link: '/nl/guide/creation-functions/basic/timer' },
          ],
        },
        {
          text: 'Lus Generatie',
          link: '/nl/guide/creation-functions/loop/',
          collapsed: true,
          items: [
            { text: 'range', link: '/nl/guide/creation-functions/loop/range' },
            { text: 'generate', link: '/nl/guide/creation-functions/loop/generate' },
          ],
        },
        {
          text: 'HTTP Communicatie',
          link: '/nl/guide/creation-functions/http-communication/',
          collapsed: true,
          items: [
            { text: 'ajax', link: '/nl/guide/creation-functions/http-communication/ajax' },
            { text: 'fromFetch', link: '/nl/guide/creation-functions/http-communication/fromFetch' },
          ],
        },
        {
          text: 'Combinatie',
          link: '/nl/guide/creation-functions/combination/',
          collapsed: true,
          items: [
            { text: 'concat', link: '/nl/guide/creation-functions/combination/concat' },
            { text: 'merge', link: '/nl/guide/creation-functions/combination/merge' },
            { text: 'combineLatest', link: '/nl/guide/creation-functions/combination/combineLatest' },
            { text: 'zip', link: '/nl/guide/creation-functions/combination/zip' },
            { text: 'forkJoin', link: '/nl/guide/creation-functions/combination/forkJoin' },
            { text: 'forkJoin vs combineLatest', link: '/nl/guide/creation-functions/combination/forkJoin-vs-combineLatest' },
          ],
        },
        {
          text: 'Selectie/Partitie',
          link: '/nl/guide/creation-functions/selection/',
          collapsed: true,
          items: [
            { text: 'race', link: '/nl/guide/creation-functions/selection/race' },
            { text: 'partition', link: '/nl/guide/creation-functions/selection/partition' },
          ],
        },
        {
          text: 'Conditionele Vertakking',
          link: '/nl/guide/creation-functions/conditional/',
          collapsed: true,
          items: [
            { text: 'iif', link: '/nl/guide/creation-functions/conditional/iif' },
            { text: 'defer', link: '/nl/guide/creation-functions/conditional/defer' },
          ],
        },
        {
          text: 'Besturing',
          link: '/nl/guide/creation-functions/control/',
          collapsed: true,
          items: [
            { text: 'scheduled', link: '/nl/guide/creation-functions/control/scheduled' },
            { text: 'using', link: '/nl/guide/creation-functions/control/using' },
          ],
        },
      ],
    },
    {
      text: '4. Operators Begrijpen',
      link: '/nl/guide/operators/',
      items: [
        { text: 'Pipeline Concepten', link: '/nl/guide/operators/pipeline' },
        {
          text: 'Transformatie Operators',
          link: '/nl/guide/operators/transformation/',
          collapsed: true,
          items: [
            { text: 'map', link: '/nl/guide/operators/transformation/map' },
            { text: 'scan', link: '/nl/guide/operators/transformation/scan' },
            { text: 'mergeScan', link: '/nl/guide/operators/transformation/mergeScan' },
            { text: 'reduce', link: '/nl/guide/operators/transformation/reduce' },
            { text: 'pairwise', link: '/nl/guide/operators/transformation/pairwise' },
            { text: 'groupBy', link: '/nl/guide/operators/transformation/groupBy' },
            {
              text: 'mergeMap',
              link: '/nl/guide/operators/transformation/mergeMap',
            },
            {
              text: 'switchMap',
              link: '/nl/guide/operators/transformation/switchMap',
            },
            {
              text: 'concatMap',
              link: '/nl/guide/operators/transformation/concatMap',
            },
            {
              text: 'exhaustMap',
              link: '/nl/guide/operators/transformation/exhaustMap',
            },
            {
              text: 'expand',
              link: '/nl/guide/operators/transformation/expand',
            },
            {
              text: 'buffer',
              link: '/nl/guide/operators/transformation/buffer',
            },
            {
              text: 'bufferTime',
              link: '/nl/guide/operators/transformation/bufferTime',
            },
            {
              text: 'bufferCount',
              link: '/nl/guide/operators/transformation/bufferCount',
            },
            {
              text: 'bufferWhen',
              link: '/nl/guide/operators/transformation/bufferWhen',
            },
            {
              text: 'bufferToggle',
              link: '/nl/guide/operators/transformation/bufferToggle',
            },
            {
              text: 'windowTime',
              link: '/nl/guide/operators/transformation/windowTime',
            },
            {
              text: 'window',
              link: '/nl/guide/operators/transformation/window',
            },
            {
              text: 'windowCount',
              link: '/nl/guide/operators/transformation/windowCount',
            },
            {
              text: 'windowToggle',
              link: '/nl/guide/operators/transformation/windowToggle',
            },
            {
              text: 'windowWhen',
              link: '/nl/guide/operators/transformation/windowWhen',
            },
            {
              text: 'Praktische Gebruiksscenario\'s',
              link: '/nl/guide/operators/transformation/practical-use-cases.md',
            },
          ],
        },
        {
          text: 'Filter Operators',
          link: '/nl/guide/operators/filtering/',
          collapsed: true,
          items: [
            { text: 'filter', link: '/nl/guide/operators/filtering/filter' },
            { text: 'take', link: '/nl/guide/operators/filtering/take' },
            { text: 'takeLast', link: '/nl/guide/operators/filtering/takeLast' },
            { text: 'takeWhile', link: '/nl/guide/operators/filtering/takeWhile' },
            { text: 'skip', link: '/nl/guide/operators/filtering/skip' },
            { text: 'skipLast', link: '/nl/guide/operators/filtering/skipLast' },
            { text: 'skipWhile', link: '/nl/guide/operators/filtering/skipWhile' },
            { text: 'skipUntil', link: '/nl/guide/operators/filtering/skipUntil' },
            { text: 'first', link: '/nl/guide/operators/filtering/first' },
            { text: 'last', link: '/nl/guide/operators/filtering/last' },
            { text: 'elementAt', link: '/nl/guide/operators/filtering/elementAt' },
            { text: 'find', link: '/nl/guide/operators/filtering/find' },
            { text: 'findIndex', link: '/nl/guide/operators/filtering/findIndex' },
            {
              text: 'debounceTime',
              link: '/nl/guide/operators/filtering/debounceTime',
            },
            {
              text: 'throttleTime',
              link: '/nl/guide/operators/filtering/throttleTime',
            },
            {
              text: 'auditTime',
              link: '/nl/guide/operators/filtering/auditTime',
            },
            { text: 'audit', link: '/nl/guide/operators/filtering/audit' },
            { text: 'sampleTime', link: '/nl/guide/operators/filtering/sampleTime' },
            { text: 'ignoreElements', link: '/nl/guide/operators/filtering/ignoreElements' },
            {
              text: 'distinct',
              link: '/nl/guide/operators/filtering/distinct',
            },
            {
              text: 'distinctUntilChanged',
              link: '/nl/guide/operators/filtering/distinctUntilChanged',
            },
            {
              text: 'distinctUntilKeyChanged',
              link: '/nl/guide/operators/filtering/distinctUntilKeyChanged',
            },
            {
              text: 'Praktische Gebruiksscenario\'s',
              link: '/nl/guide/operators/filtering/practical-use-cases.md',
            },
          ],
        },
        {
          text: 'Combinatie Operators (Pipeable)',
          link: '/nl/guide/operators/combination/',
          collapsed: true,
          items: [
            {
              text: 'concatWith',
              link: '/nl/guide/operators/combination/concatWith',
            },
            {
              text: 'mergeWith',
              link: '/nl/guide/operators/combination/mergeWith',
            },
            {
              text: 'combineLatestWith',
              link: '/nl/guide/operators/combination/combineLatestWith',
            },
            {
              text: 'zipWith',
              link: '/nl/guide/operators/combination/zipWith',
            },
            {
              text: 'raceWith',
              link: '/nl/guide/operators/combination/raceWith',
            },
            {
              text: 'withLatestFrom',
              link: '/nl/guide/operators/combination/withLatestFrom',
            },
            {
              text: 'mergeAll',
              link: '/nl/guide/operators/combination/mergeAll',
            },
            {
              text: 'concatAll',
              link: '/nl/guide/operators/combination/concatAll',
            },
            {
              text: 'switchAll',
              link: '/nl/guide/operators/combination/switchAll',
            },
            {
              text: 'exhaustAll',
              link: '/nl/guide/operators/combination/exhaustAll',
            },
            {
              text: 'combineLatestAll',
              link: '/nl/guide/operators/combination/combineLatestAll',
            },
            {
              text: 'zipAll',
              link: '/nl/guide/operators/combination/zipAll',
            },
            {
              text: 'Praktische Gebruiksscenario\'s',
              link: '/nl/guide/operators/combination/practical-use-cases.md',
            },
          ],
        },
        {
          text: 'Utility Operators',
          link: '/nl/guide/operators/utility/',
          collapsed: true,
          items: [
            { text: 'tap', link: '/nl/guide/operators/utility/tap' },
            { text: 'delay', link: '/nl/guide/operators/utility/delay' },
            { text: 'delayWhen', link: '/nl/guide/operators/utility/delayWhen' },
            { text: 'timeout', link: '/nl/guide/operators/utility/timeout' },
            { text: 'takeUntil', link: '/nl/guide/operators/utility/takeUntil' },
            { text: 'finalize', link: '/nl/guide/operators/utility/finalize' },
            { text: 'repeat', link: '/nl/guide/operators/utility/repeat' },
            { text: 'retry', link: '/nl/guide/operators/utility/retry' },
            { text: 'startWith', link: '/nl/guide/operators/utility/startWith' },
            { text: 'toArray', link: '/nl/guide/operators/utility/toArray' },
            { text: 'materialize', link: '/nl/guide/operators/utility/materialize' },
            { text: 'dematerialize', link: '/nl/guide/operators/utility/dematerialize' },
            { text: 'observeOn', link: '/nl/guide/operators/utility/observeOn' },
            { text: 'subscribeOn', link: '/nl/guide/operators/utility/subscribeOn' },
            { text: 'timestamp', link: '/nl/guide/operators/utility/timestamp' },
            {
              text: 'Praktische Gebruiksscenario\'s',
              link: '/nl/guide/operators/utility/practical-use-cases.md',
            },
          ],
        },
        {
          text: 'Conditionele Operators',
          link: '/nl/guide/operators/conditional/',
          collapsed: true,
          items: [
            {
              text: 'defaultIfEmpty',
              link: '/nl/guide/operators/conditional/defaultIfEmpty',
            },
            { text: 'every', link: '/nl/guide/operators/conditional/every' },
            { text: 'isEmpty', link: '/nl/guide/operators/conditional/isEmpty' },
            {
              text: 'Praktische Gebruiksscenario\'s',
              link: '/nl/guide/operators/conditional/practical-use-cases.md',
            },
          ],
        },
        {
          text: 'Multicasting',
          link: '/nl/guide/operators/multicasting/',
          collapsed: true,
          items: [
            { text: 'share', link: '/nl/guide/operators/multicasting/share' },
            { text: 'shareReplay', link: '/nl/guide/operators/multicasting/shareReplay' },
          ],
        },
      ],
    },
    {
      text: '5. Subject en Multicast',
      items: [
        { text: 'Wat is Subject?', link: '/nl/guide/subjects/what-is-subject' },
        { text: 'Types van Subject', link: '/nl/guide/subjects/types-of-subject' },
        {
          text: 'Hoe Multicasting Werkt',
          link: '/nl/guide/subjects/multicasting',
        },
        { text: 'Gebruiksscenario\'s van Subject', link: '/nl/guide/subjects/use-cases' },
      ],
    },
    {
      text: '6. Foutafhandeling',
      items: [
        { text: 'Foutafhandelingsstrategieën', link: '/nl/guide/error-handling/strategies' },
        {
          text: 'Twee Plaatsen voor Foutafhandeling',
          link: '/nl/guide/error-handling/error-handling-locations',
        },
        {
          text: 'try-catch en RxJS Integratie',
          link: '/nl/guide/error-handling/try-catch-integration',
        },
        {
          text: 'retry en catchError',
          link: '/nl/guide/error-handling/retry-catch',
        },
        {
          text: 'finalize en complete',
          link: '/nl/guide/error-handling/finalize',
        },
      ],
    },
    {
      text: '7. Gebruik van Scheduler',
      items: [
        { text: 'Controle van Asynchrone Verwerking', link: '/nl/guide/schedulers/async-control' },
        {
          text: 'Types Schedulers en Hoe te Gebruiken',
          link: '/nl/guide/schedulers/types',
        },
        {
          text: 'Aanvulling: Basiskennis van Taken en Schedulers',
          link: '/nl/guide/schedulers/task-and-scheduler-basics',
        },
      ],
    },
    {
      text: '8. Debugtechnieken voor RxJS',
      items: [
        { text: 'Overzicht van Debugtechnieken', link: '/nl/guide/debugging/' },
        { text: 'Veelvoorkomende Debug Scenario\'s', link: '/nl/guide/debugging/common-scenarios' },
        { text: 'Aangepaste Debug Tools', link: '/nl/guide/debugging/custom-tools' },
        { text: 'Prestatie Debugging', link: '/nl/guide/debugging/performance' },
      ],
    },
    {
      text: '9. Testmethoden',
      items: [
        { text: 'RxJS Unit Testing', link: '/nl/guide/testing/unit-tests' },
        {
          text: 'Gebruik van TestScheduler',
          link: '/nl/guide/testing/test-scheduler',
        },
        { text: 'Marble Testing', link: '/nl/guide/testing/marble-testing' },
      ],
    },
    {
      text: '10. Verzameling van RxJS Anti-Patronen',
      items: [
        { text: 'Wat zijn Anti-Patronen?', link: '/nl/guide/anti-patterns/' },
        { text: 'Veelgemaakte Fouten en Hoe Ermee Om te Gaan', link: '/nl/guide/anti-patterns/common-mistakes' },
        { text: 'Wildgroei van State Management Flags', link: '/nl/guide/anti-patterns/flag-management' },
        { text: 'Hel van Geneste if-Statements in subscribe', link: '/nl/guide/anti-patterns/subscribe-if-hell' },
        { text: 'Promise en Observable Mengen', link: '/nl/guide/anti-patterns/promise-observable-mixing' },
        { text: 'One-Liner Hel en Fase Scheiding Syntax', link: '/nl/guide/anti-patterns/one-liner-hell' },
        { text: 'Checklist voor Anti-Patronen Vermijden', link: '/nl/guide/anti-patterns/checklist' },
      ],
    },
    {
      text: '11. RxJS Moeilijkheden Overwinnen',
      items: [
        { text: 'Waarom is RxJS zo Moeilijk?', link: '/nl/guide/overcoming-difficulties/' },
        { text: 'Barrière Conceptueel Begrip', link: '/nl/guide/overcoming-difficulties/conceptual-understanding' },
        { text: 'Barrière Levenscyclusbeheer', link: '/nl/guide/overcoming-difficulties/lifecycle-management' },
        { text: 'Verwarring bij Operator Selectie', link: '/nl/guide/overcoming-difficulties/operator-selection' },
        { text: 'Timing en Volgorde Begrijpen', link: '/nl/guide/overcoming-difficulties/timing-and-order' },
        { text: 'Moeilijkheid bij State Management', link: '/nl/guide/overcoming-difficulties/state-and-sharing' },
        { text: 'Meerdere Stream Combinaties', link: '/nl/guide/overcoming-difficulties/stream-combination' },
        { text: 'Debug Barrières', link: '/nl/guide/overcoming-difficulties/debugging-guide' },
      ],
    },
    {
      text: '12. Geavanceerde Integratie TypeScript en RxJS',
      // items: [
      //   {
      //     text: 'Basis TypeScript en RxJS Integratie',
      //     link: '/nl/guide/typescript-advanced/type-safety',
      //   },
      //   {
      //     text: 'Gebruik van Generics',
      //     link: '/nl/guide/typescript-advanced/generics',
      //   },
      //   {
      //     text: 'Aangepaste Operators en Typedefinities',
      //     link: '/nl/guide/typescript-advanced/custom-operators',
      //   },
      //   {
      //     text: 'Gebruik van Conditionele en Mapped Types',
      //     link: '/nl/guide/typescript-advanced/conditional-types',
      //   },
      // ],
    },
    {
      text: '13. Praktische Patronen',
      items: [
        { text: 'Overzicht van Praktische Patronen', link: '/nl/guide/practical-patterns/' },
        { text: 'UI Gebeurtenisafhandeling', link: '/nl/guide/practical-patterns/ui-events' },
        { text: 'API Aanroepen', link: '/nl/guide/practical-patterns/api-calls' },
        { text: 'Formulierverwerking', link: '/nl/guide/practical-patterns/form-handling' },
        { text: 'Geavanceerde Formulierpatronen', link: '/nl/guide/practical-patterns/advanced-form-patterns' },
        { text: 'Real-Time Dataverwerking', link: '/nl/guide/practical-patterns/real-time-data' },
        { text: 'Caching Strategieën', link: '/nl/guide/practical-patterns/caching-strategies' },
        { text: 'Foutafhandeling Praktijken', link: '/nl/guide/practical-patterns/error-handling-patterns' },
        { text: 'Conditionele Vertakkingen in subscribe', link: '/nl/guide/practical-patterns/subscribe-branching' },
      ],
    },
    {
      text: '14. Prestatieoptimalisatie',
      // items: [
      //   {
      //     text: 'Correct Abonnementsbeheer',
      //     link: '/nl/guide/performance/subscription-management',
      //   },
      //   {
      //     text: 'Efficiënte Operator Selectie',
      //     link: '/nl/guide/performance/operator-selection',
      //   },
      //   {
      //     text: 'Stream Ontwerppatronen',
      //     link: '/nl/guide/performance/stream-design',
      //   },
      // ],
    },
    {
      text: '15. Framework Integratie',
      // items: [
      //   { text: 'Integratie met Angular', link: '/nl/guide/frameworks/angular' },
      //   { text: 'Integratie met React', link: '/nl/guide/frameworks/react' },
      //   { text: 'Integratie met Vue', link: '/nl/guide/frameworks/vue' },
      //   { text: 'Integratie met Web API', link: '/nl/guide/frameworks/web-api' },
      // ],
    },
    {
      text: 'Bijlage',
      items: [
        { text: 'Bijlage Overzicht', link: '/nl/guide/appendix/' },
        { text: 'Embedded Ontwikkeling en Reactief Programmeren', link: '/nl/guide/appendix/embedded-reactive-programming' },
        { text: 'Reactieve Methoden buiten ReactiveX', link: '/nl/guide/appendix/reactive-patterns-beyond-rxjs' },
        { text: 'Overzicht van Reactieve Architecturen', link: '/nl/guide/appendix/reactive-architecture-map' },
        { text: 'Reactief Programmeren Heroverwogen', link: '/nl/guide/appendix/reactive-programming-reconsidered' },
        { text: 'RxJS en Reactive Streams Ecosysteem', link: '/nl/guide/appendix/rxjs-and-reactive-streams-ecosystem' },
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
    message: 'Uitgebracht onder de CC-BY-4.0 licentie.',
    copyright: 'Copyright © 2025 shuji-bonji',
  },
};
