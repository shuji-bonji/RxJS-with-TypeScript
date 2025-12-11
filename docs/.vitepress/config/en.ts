import { DefaultTheme } from 'vitepress';

export const enThemeConfig: DefaultTheme.Config = {
  nav: [
    { text: 'Home', link: '/en/' },
    { text: 'Guide', link: '/en/guide/' },
  ],
  sidebar: [
    {
      text: '1. Introduction to RxJS',
      items: [
        { text: 'Introduction', link: '/en/guide/introduction' },
        { text: 'Building Learning Execution Environment', link: '/en/guide/starter-kid.md' },
        { text: 'What is RxJS?', link: '/en/guide/basics/what-is-rxjs' },
        { text: 'What is a Stream?', link: '/en/guide/basics/what-is-a-stream' },
        { text: 'Difference between Promise and RxJS', link: '/en/guide/basics/promise-vs-rxjs' },
      ],
    },
    {
      text: '2. Basics of Observable',
      items: [
        {
          text: 'What is an Observable?',
          link: '/en/guide/observables/what-is-observable',
        },
        { text: 'How to Create an Observable', link: '/en/guide/observables/creation' },
        { text: 'Streaming Events', link: '/en/guide/observables/events' },
        {
          text: 'Events Not Available in fromEvent',
          link: '/en/guide/observables/events#cannot-used-fromEvent'
        },
        { text: 'List of Events', link: '/en/guide/observables/events-list' },
        {
          text: 'Difference between Observer and Subscriber',
          link: '/en/guide/observables/observer-vs-subscriber',
        },
        {
          text: 'Observable Lifecycle',
          link: '/en/guide/observables/observable-lifecycle',
        },
        {
          text: 'Cold Observable and Hot Observable',
          link: '/en/guide/observables/cold-and-hot-observables',
        },
      ],
    },
    {
      text: '3. Creation Functions',
      link: '/en/guide/creation-functions/',
      items: [
        {
          text: 'Basic Creation',
          link: '/en/guide/creation-functions/basic/',
          collapsed: true,
          items: [
            { text: 'of', link: '/en/guide/creation-functions/basic/of' },
            { text: 'from', link: '/en/guide/creation-functions/basic/from' },
            { text: 'fromEvent', link: '/en/guide/creation-functions/basic/fromEvent' },
            { text: 'interval', link: '/en/guide/creation-functions/basic/interval' },
            { text: 'timer', link: '/en/guide/creation-functions/basic/timer' },
          ],
        },
        {
          text: 'Loop Generation',
          link: '/en/guide/creation-functions/loop/',
          collapsed: true,
          items: [
            { text: 'range', link: '/en/guide/creation-functions/loop/range' },
            { text: 'generate', link: '/en/guide/creation-functions/loop/generate' },
          ],
        },
        {
          text: 'HTTP Communication',
          link: '/en/guide/creation-functions/http-communication/',
          collapsed: true,
          items: [
            { text: 'ajax', link: '/en/guide/creation-functions/http-communication/ajax' },
            { text: 'fromFetch', link: '/en/guide/creation-functions/http-communication/fromFetch' },
          ],
        },
        {
          text: 'Combination',
          link: '/en/guide/creation-functions/combination/',
          collapsed: true,
          items: [
            { text: 'concat', link: '/en/guide/creation-functions/combination/concat' },
            { text: 'merge', link: '/en/guide/creation-functions/combination/merge' },
            { text: 'combineLatest', link: '/en/guide/creation-functions/combination/combineLatest' },
            { text: 'zip', link: '/en/guide/creation-functions/combination/zip' },
            { text: 'forkJoin', link: '/en/guide/creation-functions/combination/forkJoin' },
            { text: 'forkJoin vs combineLatest', link: '/en/guide/creation-functions/combination/forkJoin-vs-combineLatest' },
          ],
        },
        {
          text: 'Selection/Partition',
          link: '/en/guide/creation-functions/selection/',
          collapsed: true,
          items: [
            { text: 'race', link: '/en/guide/creation-functions/selection/race' },
            { text: 'partition', link: '/en/guide/creation-functions/selection/partition' },
          ],
        },
        {
          text: 'Conditional Branching',
          link: '/en/guide/creation-functions/conditional/',
          collapsed: true,
          items: [
            { text: 'iif', link: '/en/guide/creation-functions/conditional/iif' },
            { text: 'defer', link: '/en/guide/creation-functions/conditional/defer' },
          ],
        },
        {
          text: 'Control',
          link: '/en/guide/creation-functions/control/',
          collapsed: true,
          items: [
            { text: 'scheduled', link: '/en/guide/creation-functions/control/scheduled' },
            { text: 'using', link: '/en/guide/creation-functions/control/using' },
          ],
        },
      ],
    },
    {
      text: '4. Understanding Operators',
      link: '/en/guide/operators/',
      items: [
        { text: 'Pipeline Concepts', link: '/en/guide/operators/pipeline' },
        {
          text: 'Transformation Operators',
          link: '/en/guide/operators/transformation/',
          collapsed: true,
          items: [
            { text: 'map', link: '/en/guide/operators/transformation/map' },
            { text: 'scan', link: '/en/guide/operators/transformation/scan' },
            { text: 'mergeScan', link: '/en/guide/operators/transformation/mergeScan' },
            { text: 'reduce', link: '/en/guide/operators/transformation/reduce' },
            { text: 'pairwise', link: '/en/guide/operators/transformation/pairwise' },
            { text: 'groupBy', link: '/en/guide/operators/transformation/groupBy' },
            {
              text: 'mergeMap',
              link: '/en/guide/operators/transformation/mergeMap',
            },
            {
              text: 'switchMap',
              link: '/en/guide/operators/transformation/switchMap',
            },
            {
              text: 'concatMap',
              link: '/en/guide/operators/transformation/concatMap',
            },
            {
              text: 'exhaustMap',
              link: '/en/guide/operators/transformation/exhaustMap',
            },
            {
              text: 'expand',
              link: '/en/guide/operators/transformation/expand',
            },
            {
              text: 'buffer',
              link: '/en/guide/operators/transformation/buffer',
            },
            {
              text: 'bufferTime',
              link: '/en/guide/operators/transformation/bufferTime',
            },
            {
              text: 'bufferCount',
              link: '/en/guide/operators/transformation/bufferCount',
            },
            {
              text: 'bufferWhen',
              link: '/en/guide/operators/transformation/bufferWhen',
            },
            {
              text: 'bufferToggle',
              link: '/en/guide/operators/transformation/bufferToggle',
            },
            {
              text: 'windowTime',
              link: '/en/guide/operators/transformation/windowTime',
            },
            {
              text: 'window',
              link: '/en/guide/operators/transformation/window',
            },
            {
              text: 'windowCount',
              link: '/en/guide/operators/transformation/windowCount',
            },
            {
              text: 'windowToggle',
              link: '/en/guide/operators/transformation/windowToggle',
            },
            {
              text: 'windowWhen',
              link: '/en/guide/operators/transformation/windowWhen',
            },
            {
              text: 'Practical Use Cases',
              link: '/en/guide/operators/transformation/practical-use-cases.md',
            },
          ],
        },
        {
          text: 'Filtering Operators',
          link: '/en/guide/operators/filtering/',
          collapsed: true,
          items: [
            { text: 'filter', link: '/en/guide/operators/filtering/filter' },
            { text: 'take', link: '/en/guide/operators/filtering/take' },
            { text: 'takeLast', link: '/en/guide/operators/filtering/takeLast' },
            { text: 'takeWhile', link: '/en/guide/operators/filtering/takeWhile' },
            { text: 'skip', link: '/en/guide/operators/filtering/skip' },
            { text: 'skipLast', link: '/en/guide/operators/filtering/skipLast' },
            { text: 'skipWhile', link: '/en/guide/operators/filtering/skipWhile' },
            { text: 'skipUntil', link: '/en/guide/operators/filtering/skipUntil' },
            { text: 'first', link: '/en/guide/operators/filtering/first' },
            { text: 'last', link: '/en/guide/operators/filtering/last' },
            { text: 'elementAt', link: '/en/guide/operators/filtering/elementAt' },
            { text: 'find', link: '/en/guide/operators/filtering/find' },
            { text: 'findIndex', link: '/en/guide/operators/filtering/findIndex' },
            {
              text: 'debounceTime',
              link: '/en/guide/operators/filtering/debounceTime',
            },
            {
              text: 'throttleTime',
              link: '/en/guide/operators/filtering/throttleTime',
            },
            {
              text: 'auditTime',
              link: '/en/guide/operators/filtering/auditTime',
            },
            { text: 'audit', link: '/en/guide/operators/filtering/audit' },
            { text: 'sampleTime', link: '/en/guide/operators/filtering/sampleTime' },
            { text: 'ignoreElements', link: '/en/guide/operators/filtering/ignoreElements' },
            {
              text: 'distinct',
              link: '/en/guide/operators/filtering/distinct',
            },
            {
              text: 'distinctUntilChanged',
              link: '/en/guide/operators/filtering/distinctUntilChanged',
            },
            {
              text: 'distinctUntilKeyChanged',
              link: '/en/guide/operators/filtering/distinctUntilKeyChanged',
            },
            {
              text: 'Practical Use Cases',
              link: '/en/guide/operators/filtering/practical-use-cases.md',
            },
          ],
        },
        {
          text: 'Combination Operators (Pipeable)',
          link: '/en/guide/operators/combination/',
          collapsed: true,
          items: [
            {
              text: 'concatWith',
              link: '/en/guide/operators/combination/concatWith',
            },
            {
              text: 'mergeWith',
              link: '/en/guide/operators/combination/mergeWith',
            },
            {
              text: 'combineLatestWith',
              link: '/en/guide/operators/combination/combineLatestWith',
            },
            {
              text: 'zipWith',
              link: '/en/guide/operators/combination/zipWith',
            },
            {
              text: 'raceWith',
              link: '/en/guide/operators/combination/raceWith',
            },
            {
              text: 'withLatestFrom',
              link: '/en/guide/operators/combination/withLatestFrom',
            },
            {
              text: 'mergeAll',
              link: '/en/guide/operators/combination/mergeAll',
            },
            {
              text: 'concatAll',
              link: '/en/guide/operators/combination/concatAll',
            },
            {
              text: 'switchAll',
              link: '/en/guide/operators/combination/switchAll',
            },
            {
              text: 'exhaustAll',
              link: '/en/guide/operators/combination/exhaustAll',
            },
            {
              text: 'combineLatestAll',
              link: '/en/guide/operators/combination/combineLatestAll',
            },
            {
              text: 'zipAll',
              link: '/en/guide/operators/combination/zipAll',
            },
            {
              text: 'Practical Use Cases',
              link: '/en/guide/operators/combination/practical-use-cases.md',
            },
          ],
        },
        {
          text: 'Utility Operators',
          link: '/en/guide/operators/utility/',
          collapsed: true,
          items: [
            { text: 'tap', link: '/en/guide/operators/utility/tap' },
            { text: 'delay', link: '/en/guide/operators/utility/delay' },
            { text: 'delayWhen', link: '/en/guide/operators/utility/delayWhen' },
            { text: 'timeout', link: '/en/guide/operators/utility/timeout' },
            { text: 'takeUntil', link: '/en/guide/operators/utility/takeUntil' },
            { text: 'finalize', link: '/en/guide/operators/utility/finalize' },
            { text: 'repeat', link: '/en/guide/operators/utility/repeat' },
            { text: 'retry', link: '/en/guide/operators/utility/retry' },
            { text: 'startWith', link: '/en/guide/operators/utility/startWith' },
            { text: 'toArray', link: '/en/guide/operators/utility/toArray' },
            { text: 'materialize', link: '/en/guide/operators/utility/materialize' },
            { text: 'dematerialize', link: '/en/guide/operators/utility/dematerialize' },
            { text: 'observeOn', link: '/en/guide/operators/utility/observeOn' },
            { text: 'subscribeOn', link: '/en/guide/operators/utility/subscribeOn' },
            { text: 'timestamp', link: '/en/guide/operators/utility/timestamp' },
            {
              text: 'Practical Use Cases',
              link: '/en/guide/operators/utility/practical-use-cases.md',
            },
          ],
        },
        {
          text: 'Conditional Operators',
          link: '/en/guide/operators/conditional/',
          collapsed: true,
          items: [
            {
              text: 'defaultIfEmpty',
              link: '/en/guide/operators/conditional/defaultIfEmpty',
            },
            { text: 'every', link: '/en/guide/operators/conditional/every' },
            { text: 'isEmpty', link: '/en/guide/operators/conditional/isEmpty' },
            {
              text: 'Practical Use Cases',
              link: '/en/guide/operators/conditional/practical-use-cases.md',
            },
          ],
        },
        {
          text: 'Multicasting',
          link: '/en/guide/operators/multicasting/',
          collapsed: true,
          items: [
            { text: 'share', link: '/en/guide/operators/multicasting/share' },
            { text: 'shareReplay', link: '/en/guide/operators/multicasting/shareReplay' },
          ],
        },
      ],
    },
    {
      text: '5. Subject and Multicast',
      items: [
        { text: 'What is Subject?', link: '/en/guide/subjects/what-is-subject' },
        { text: 'Types of Subject', link: '/en/guide/subjects/types-of-subject' },
        {
          text: 'How Multicasting Works',
          link: '/en/guide/subjects/multicasting',
        },
        { text: 'Use Cases of Subject', link: '/en/guide/subjects/use-cases' },
      ],
    },
    {
      text: '6. Error Handling',
      items: [
        { text: 'Error Handling Strategies', link: '/en/guide/error-handling/strategies' },
        {
          text: 'Two Places for Error Handling',
          link: '/en/guide/error-handling/error-handling-locations',
        },
        {
          text: 'try-catch and RxJS Integration',
          link: '/en/guide/error-handling/try-catch-integration',
        },
        {
          text: 'retry and catchError',
          link: '/en/guide/error-handling/retry-catch',
        },
        {
          text: 'finalize and complete',
          link: '/en/guide/error-handling/finalize',
        },
      ],
    },
    {
      text: '7. Use of Scheduler',
      items: [
        { text: 'Control of Asynchronous Processing', link: '/en/guide/schedulers/async-control' },
        {
          text: 'Types of Schedulers and How to Use Them',
          link: '/en/guide/schedulers/types',
        },
        {
          text: 'Supplement: Basic Knowledge of Tasks and Schedulers',
          link: '/en/guide/schedulers/task-and-scheduler-basics',
        },
      ],
    },
    {
      text: '8. Debugging Techniques for RxJS',
      items: [
        { text: 'Overview of Debugging Techniques', link: '/en/guide/debugging/' },
        { text: 'Common Debugging Scenarios', link: '/en/guide/debugging/common-scenarios' },
        { text: 'Custom Debugging Tools', link: '/en/guide/debugging/custom-tools' },
        { text: 'Performance Debugging', link: '/en/guide/debugging/performance' },
      ],
    },
    {
      text: '9. Testing Methods',
      items: [
        { text: 'RxJS Unit Testing', link: '/en/guide/testing/unit-tests' },
        {
          text: 'Utilization of TestScheduler',
          link: '/en/guide/testing/test-scheduler',
        },
        { text: 'Marble Testing', link: '/en/guide/testing/marble-testing' },
      ],
    },
    {
      text: '10. Collection of RxJS Anti-Patterns',
      items: [
        { text: 'What are Anti-Patterns?', link: '/en/guide/anti-patterns/' },
        { text: 'Common Mistakes and How to Deal with Them', link: '/en/guide/anti-patterns/common-mistakes' },
        { text: 'Rampant State Management Flags', link: '/en/guide/anti-patterns/flag-management' },
        { text: 'Hell of Nesting if Statements in subscribe', link: '/en/guide/anti-patterns/subscribe-if-hell' },
        { text: 'Mixing Promise and Observable', link: '/en/guide/anti-patterns/promise-observable-mixing' },
        { text: 'One-Liner Hell and Stage Separation Syntax', link: '/en/guide/anti-patterns/one-liner-hell' },
        { text: 'Checklist for Avoiding Anti-Patterns', link: '/en/guide/anti-patterns/checklist' },
      ],
    },
    {
      text: '11. Overcoming RxJS Difficulties',
      items: [
        { text: 'Why is RxJS so Difficult?', link: '/en/guide/overcoming-difficulties/' },
        { text: 'Conceptual Understanding Barrier', link: '/en/guide/overcoming-difficulties/conceptual-understanding' },
        { text: 'Lifecycle Management Barrier', link: '/en/guide/overcoming-difficulties/lifecycle-management' },
        { text: 'Confusion in Operator Selection', link: '/en/guide/overcoming-difficulties/operator-selection' },
        { text: 'Understanding Timing and Sequence', link: '/en/guide/overcoming-difficulties/timing-and-order' },
        { text: 'Difficulty in State Management', link: '/en/guide/overcoming-difficulties/state-and-sharing' },
        { text: 'Multiple Stream Combinations', link: '/en/guide/overcoming-difficulties/stream-combination' },
        { text: 'Debugging Barriers', link: '/en/guide/overcoming-difficulties/debugging-guide' },
      ],
    },
    {
      text: '12. Advanced Integration Between TypeScript and RxJS',
      // items: [
      //   {
      //     text: 'Basic TypeScript and RxJS Integration',
      //     link: '/en/guide/typescript-advanced/type-safety',
      //   },
      //   {
      //     text: 'Utilization of Generics',
      //     link: '/en/guide/typescript-advanced/generics',
      //   },
      //   {
      //     text: 'Custom Operators and Type Definitions',
      //     link: '/en/guide/typescript-advanced/custom-operators',
      //   },
      //   {
      //     text: 'Utilization of Conditional and Mapped Types',
      //     link: '/en/guide/typescript-advanced/conditional-types',
      //   },
      // ],
    },
    {
      text: '13. Practical Patterns',
      items: [
        { text: 'Overview of Practical Patterns', link: '/en/guide/practical-patterns/' },
        { text: 'UI Event Handling', link: '/en/guide/practical-patterns/ui-events' },
        { text: 'API Calls', link: '/en/guide/practical-patterns/api-calls' },
        { text: 'Form Processing', link: '/en/guide/practical-patterns/form-handling' },
        { text: 'Advanced Form Patterns', link: '/en/guide/practical-patterns/advanced-form-patterns' },
        { text: 'Real-Time Data Processing', link: '/en/guide/practical-patterns/real-time-data' },
        { text: 'Caching Strategies', link: '/en/guide/practical-patterns/caching-strategies' },
        { text: 'Error Handling Practices', link: '/en/guide/practical-patterns/error-handling-patterns' },
        { text: 'Conditional Branches in subscribe', link: '/en/guide/practical-patterns/subscribe-branching' },
      ],
    },
    {
      text: '14. Performance Optimization',
      // items: [
      //   {
      //     text: 'Proper Subscription Management',
      //     link: '/en/guide/performance/subscription-management',
      //   },
      //   {
      //     text: 'Efficient Operator Selection',
      //     link: '/en/guide/performance/operator-selection',
      //   },
      //   {
      //     text: 'Stream Design Patterns',
      //     link: '/en/guide/performance/stream-design',
      //   },
      // ],
    },
    {
      text: '15. Framework Integration',
      // items: [
      //   { text: 'Integration with Angular', link: '/en/guide/frameworks/angular' },
      //   { text: 'Integration with React', link: '/en/guide/frameworks/react' },
      //   { text: 'Integration with Vue', link: '/en/guide/frameworks/vue' },
      //   { text: 'Integration with Web API', link: '/en/guide/frameworks/web-api' },
      // ],
    },
    {
      text: 'Appendix',
      items: [
        { text: 'Appendix Overview', link: '/en/guide/appendix/' },
        { text: 'Embedded Development and Reactive Programming', link: '/en/guide/appendix/embedded-reactive-programming' },
        { text: 'Reactive Methods other than ReactiveX', link: '/en/guide/appendix/reactive-patterns-beyond-rxjs' },
        { text: 'Overview of Reactive Architectures', link: '/en/guide/appendix/reactive-architecture-map' },
        { text: 'Reactive Programming Reconsidered', link: '/en/guide/appendix/reactive-programming-reconsidered' },
        { text: 'RxJS and Reactive Streams Ecosystem', link: '/en/guide/appendix/rxjs-and-reactive-streams-ecosystem' },
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
    message: 'Released under the CC-BY-4.0 license.',
    copyright: 'Copyright © 2025 shuji-bonji',
  },
};
