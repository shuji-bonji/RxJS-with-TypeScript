import { DefaultTheme } from 'vitepress';

export const esThemeConfig: DefaultTheme.Config = {
  nav: [
    { text: 'Inicio', link: '/es/' },
    { text: 'Guía', link: '/es/guide/' },
  ],
  sidebar: [
    {
      text: '1. Introducción a RxJS',
      items: [
        { text: 'Introducción', link: '/es/guide/introduction' },
        { text: 'Configuración del entorno', link: '/es/guide/starter-kid' },
        { text: '¿Qué es RxJS?', link: '/es/guide/basics/what-is-rxjs' },
        { text: '¿Qué es un Stream?', link: '/es/guide/basics/what-is-a-stream' },
        { text: 'Diferencia entre Promise y RxJS', link: '/es/guide/basics/promise-vs-rxjs' },
      ],
    },
    {
      text: '2. Fundamentos de Observable',
      items: [
        { text: '¿Qué es un Observable?', link: '/es/guide/observables/what-is-observable' },
        { text: 'Crear un Observable', link: '/es/guide/observables/creation' },
        { text: 'Convertir eventos en streams', link: '/es/guide/observables/events' },
        { text: 'Lista de eventos', link: '/es/guide/observables/events-list' },
        { text: 'Observer vs Subscriber', link: '/es/guide/observables/observer-vs-subscriber' },
        { text: 'Ciclo de vida de Observable', link: '/es/guide/observables/observable-lifecycle' },
        { text: 'Cold y Hot Observable', link: '/es/guide/observables/cold-and-hot-observables' },
      ],
    },
    {
      text: '3. Funciones de creación',
      link: '/es/guide/creation-functions/',
      items: [
        {
          text: 'Creación básica',
          link: '/es/guide/creation-functions/basic/',
          collapsed: true,
          items: [
            { text: 'of', link: '/es/guide/creation-functions/basic/of' },
            { text: 'from', link: '/es/guide/creation-functions/basic/from' },
            { text: 'fromEvent', link: '/es/guide/creation-functions/basic/fromEvent' },
            { text: 'interval', link: '/es/guide/creation-functions/basic/interval' },
            { text: 'timer', link: '/es/guide/creation-functions/basic/timer' },
          ],
        },
        {
          text: 'Generación en bucle',
          link: '/es/guide/creation-functions/loop/',
          collapsed: true,
          items: [
            { text: 'range', link: '/es/guide/creation-functions/loop/range' },
            { text: 'generate', link: '/es/guide/creation-functions/loop/generate' },
          ],
        },
        {
          text: 'Comunicación HTTP',
          link: '/es/guide/creation-functions/http-communication/',
          collapsed: true,
          items: [
            { text: 'ajax', link: '/es/guide/creation-functions/http-communication/ajax' },
            { text: 'fromFetch', link: '/es/guide/creation-functions/http-communication/fromFetch' },
          ],
        },
        {
          text: 'Combinación',
          link: '/es/guide/creation-functions/combination/',
          collapsed: true,
          items: [
            { text: 'concat', link: '/es/guide/creation-functions/combination/concat' },
            { text: 'merge', link: '/es/guide/creation-functions/combination/merge' },
            { text: 'combineLatest', link: '/es/guide/creation-functions/combination/combineLatest' },
            { text: 'zip', link: '/es/guide/creation-functions/combination/zip' },
            { text: 'forkJoin', link: '/es/guide/creation-functions/combination/forkJoin' },
          ],
        },
        {
          text: 'Selección/Partición',
          link: '/es/guide/creation-functions/selection/',
          collapsed: true,
          items: [
            { text: 'race', link: '/es/guide/creation-functions/selection/race' },
            { text: 'partition', link: '/es/guide/creation-functions/selection/partition' },
          ],
        },
        {
          text: 'Ramificación condicional',
          link: '/es/guide/creation-functions/conditional/',
          collapsed: true,
          items: [
            { text: 'iif', link: '/es/guide/creation-functions/conditional/iif' },
            { text: 'defer', link: '/es/guide/creation-functions/conditional/defer' },
          ],
        },
        {
          text: 'Control',
          link: '/es/guide/creation-functions/control/',
          collapsed: true,
          items: [
            { text: 'scheduled', link: '/es/guide/creation-functions/control/scheduled' },
            { text: 'using', link: '/es/guide/creation-functions/control/using' },
          ],
        },
      ],
    },
    {
      text: '4. Entender los operadores',
      link: '/es/guide/operators/',
      items: [
        { text: 'Conceptos de pipeline', link: '/es/guide/operators/pipeline' },
        {
          text: 'Operadores de transformación',
          link: '/es/guide/operators/transformation/',
          collapsed: true,
          items: [
            { text: 'map', link: '/es/guide/operators/transformation/map' },
            { text: 'scan', link: '/es/guide/operators/transformation/scan' },
            { text: 'mergeMap', link: '/es/guide/operators/transformation/mergeMap' },
            { text: 'switchMap', link: '/es/guide/operators/transformation/switchMap' },
            { text: 'concatMap', link: '/es/guide/operators/transformation/concatMap' },
            { text: 'exhaustMap', link: '/es/guide/operators/transformation/exhaustMap' },
            { text: 'Casos de uso prácticos', link: '/es/guide/operators/transformation/practical-use-cases' },
          ],
        },
        {
          text: 'Operadores de filtrado',
          link: '/es/guide/operators/filtering/',
          collapsed: true,
          items: [
            { text: 'filter', link: '/es/guide/operators/filtering/filter' },
            { text: 'take', link: '/es/guide/operators/filtering/take' },
            { text: 'debounceTime', link: '/es/guide/operators/filtering/debounceTime' },
            { text: 'throttleTime', link: '/es/guide/operators/filtering/throttleTime' },
            { text: 'distinctUntilChanged', link: '/es/guide/operators/filtering/distinctUntilChanged' },
            { text: 'Casos de uso prácticos', link: '/es/guide/operators/filtering/practical-use-cases' },
          ],
        },
        {
          text: 'Operadores de combinación',
          link: '/es/guide/operators/combination/',
          collapsed: true,
          items: [
            { text: 'withLatestFrom', link: '/es/guide/operators/combination/withLatestFrom' },
            { text: 'mergeWith', link: '/es/guide/operators/combination/mergeWith' },
            { text: 'Casos de uso prácticos', link: '/es/guide/operators/combination/practical-use-cases' },
          ],
        },
        {
          text: 'Operadores de utilidad',
          link: '/es/guide/operators/utility/',
          collapsed: true,
          items: [
            { text: 'tap', link: '/es/guide/operators/utility/tap' },
            { text: 'delay', link: '/es/guide/operators/utility/delay' },
            { text: 'takeUntil', link: '/es/guide/operators/utility/takeUntil' },
            { text: 'finalize', link: '/es/guide/operators/utility/finalize' },
            { text: 'retry', link: '/es/guide/operators/utility/retry' },
            { text: 'Casos de uso prácticos', link: '/es/guide/operators/utility/practical-use-cases' },
          ],
        },
        {
          text: 'Multicasting',
          link: '/es/guide/operators/multicasting/',
          collapsed: true,
          items: [
            { text: 'share', link: '/es/guide/operators/multicasting/share' },
            { text: 'shareReplay', link: '/es/guide/operators/multicasting/shareReplay' },
          ],
        },
      ],
    },
    {
      text: '5. Subject y Multicast',
      items: [
        { text: '¿Qué es un Subject?', link: '/es/guide/subjects/what-is-subject' },
        { text: 'Tipos de Subject', link: '/es/guide/subjects/types-of-subject' },
        { text: 'Mecanismo de Multicasting', link: '/es/guide/subjects/multicasting' },
        { text: 'Casos de uso de Subject', link: '/es/guide/subjects/use-cases' },
      ],
    },
    {
      text: '6. Manejo de errores',
      items: [
        { text: 'Estrategias de manejo de errores', link: '/es/guide/error-handling/strategies' },
        { text: 'Dos lugares para manejo de errores', link: '/es/guide/error-handling/error-handling-locations' },
        { text: 'Integración try-catch y RxJS', link: '/es/guide/error-handling/try-catch-integration' },
        { text: 'retry y catchError', link: '/es/guide/error-handling/retry-catch' },
        { text: 'finalize y complete', link: '/es/guide/error-handling/finalize' },
      ],
    },
    {
      text: '7. Uso de Schedulers',
      items: [
        { text: 'Control de procesamiento asíncrono', link: '/es/guide/schedulers/async-control' },
        { text: 'Tipos de Schedulers', link: '/es/guide/schedulers/types' },
        { text: 'Bases de tareas y Schedulers', link: '/es/guide/schedulers/task-and-scheduler-basics' },
      ],
    },
    {
      text: '8. Técnicas de depuración RxJS',
      items: [
        { text: 'Resumen de técnicas de depuración', link: '/es/guide/debugging/' },
        { text: 'Escenarios de depuración comunes', link: '/es/guide/debugging/common-scenarios' },
        { text: 'Herramientas de depuración personalizadas', link: '/es/guide/debugging/custom-tools' },
        { text: 'Depuración de rendimiento', link: '/es/guide/debugging/performance' },
      ],
    },
    {
      text: '9. Métodos de prueba',
      items: [
        { text: 'Pruebas unitarias RxJS', link: '/es/guide/testing/unit-tests' },
        { text: 'Uso de TestScheduler', link: '/es/guide/testing/test-scheduler' },
        { text: 'Marble Testing', link: '/es/guide/testing/marble-testing' },
      ],
    },
    {
      text: '10. Anti-patrones RxJS',
      items: [
        { text: '¿Qué es un anti-patrón?', link: '/es/guide/anti-patterns/' },
        { text: 'Errores comunes y soluciones', link: '/es/guide/anti-patterns/common-mistakes' },
        { text: 'Proliferación de flags', link: '/es/guide/anti-patterns/flag-management' },
        { text: 'Infierno de if en subscribe', link: '/es/guide/anti-patterns/subscribe-if-hell' },
        { text: 'Mezclar Promise y Observable', link: '/es/guide/anti-patterns/promise-observable-mixing' },
        { text: 'Infierno de one-liners', link: '/es/guide/anti-patterns/one-liner-hell' },
        { text: 'Checklist de anti-patrones', link: '/es/guide/anti-patterns/checklist' },
      ],
    },
    {
      text: '11. Superar las dificultades de RxJS',
      items: [
        { text: '¿Por qué RxJS es difícil?', link: '/es/guide/overcoming-difficulties/' },
        { text: 'Comprensión conceptual', link: '/es/guide/overcoming-difficulties/conceptual-understanding' },
        { text: 'Gestión del ciclo de vida', link: '/es/guide/overcoming-difficulties/lifecycle-management' },
        { text: 'Selección de operadores', link: '/es/guide/overcoming-difficulties/operator-selection' },
        { text: 'Timing y orden', link: '/es/guide/overcoming-difficulties/timing-and-order' },
        { text: 'Gestión del estado', link: '/es/guide/overcoming-difficulties/state-and-sharing' },
        { text: 'Combinación de streams', link: '/es/guide/overcoming-difficulties/stream-combination' },
        { text: 'Guía de depuración', link: '/es/guide/overcoming-difficulties/debugging-guide' },
      ],
    },
    {
      text: '13. Patrones prácticos',
      items: [
        { text: 'Resumen de patrones', link: '/es/guide/practical-patterns/' },
        { text: 'Procesamiento de eventos UI', link: '/es/guide/practical-patterns/ui-events' },
        { text: 'Llamadas API', link: '/es/guide/practical-patterns/api-calls' },
        { text: 'Procesamiento de formularios', link: '/es/guide/practical-patterns/form-handling' },
        { text: 'Patrones de formularios avanzados', link: '/es/guide/practical-patterns/advanced-form-patterns' },
        { text: 'Datos en tiempo real', link: '/es/guide/practical-patterns/real-time-data' },
        { text: 'Estrategias de caché', link: '/es/guide/practical-patterns/caching-strategies' },
        { text: 'Patrones de manejo de errores', link: '/es/guide/practical-patterns/error-handling-patterns' },
        { text: 'Ramificación en subscribe', link: '/es/guide/practical-patterns/subscribe-branching' },
      ],
    },
    {
      text: 'Apéndice',
      items: [
        { text: 'Resumen del apéndice', link: '/es/guide/appendix/' },
        { text: 'Desarrollo embebido y RP', link: '/es/guide/appendix/embedded-reactive-programming' },
        { text: 'Métodos reactivos más allá de ReactiveX', link: '/es/guide/appendix/reactive-patterns-beyond-rxjs' },
        { text: 'Mapa de arquitectura reactiva', link: '/es/guide/appendix/reactive-architecture-map' },
        { text: 'Programación reactiva reconsiderada', link: '/es/guide/appendix/reactive-programming-reconsidered' },
        { text: 'Ecosistema RxJS y Reactive Streams', link: '/es/guide/appendix/rxjs-and-reactive-streams-ecosystem' },
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
    message: 'Publicado bajo licencia CC-BY-4.0.',
    copyright: 'Copyright © 2025 shuji-bonji',
  },
};
