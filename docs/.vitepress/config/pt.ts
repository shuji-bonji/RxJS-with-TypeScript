import { DefaultTheme } from 'vitepress';

export const ptThemeConfig: DefaultTheme.Config = {
  nav: [
    { text: 'Início', link: '/pt/' },
    { text: 'Guia', link: '/pt/guide/' },
  ],
  sidebar: [
    {
      text: '1. Introdução ao RxJS',
      items: [
        { text: 'Introdução', link: '/pt/guide/introduction' },
        { text: 'Configurando Ambiente de Aprendizado', link: '/pt/guide/starter-kid.md' },
        { text: 'O que é RxJS?', link: '/pt/guide/basics/what-is-rxjs' },
        { text: 'O que é um Stream?', link: '/pt/guide/basics/what-is-a-stream' },
        { text: 'Diferença entre Promise e RxJS', link: '/pt/guide/basics/promise-vs-rxjs' },
      ],
    },
    {
      text: '2. Fundamentos do Observable',
      items: [
        {
          text: 'O que é um Observable?',
          link: '/pt/guide/observables/what-is-observable',
        },
        { text: 'Criando um Observable', link: '/pt/guide/observables/creation' },
        { text: 'Streaming de Eventos', link: '/pt/guide/observables/events' },
        {
          text: 'Não disponível em fromEvent',
          link: '/pt/guide/observables/events#cannot-used-fromEvent'
        },
        { text: 'Lista de Eventos', link: '/pt/guide/observables/events-list' },
        {
          text: 'Diferença entre Observer e Subscriber',
          link: '/pt/guide/observables/observer-vs-subscriber',
        },
        {
          text: 'Ciclo de Vida do Observable',
          link: '/pt/guide/observables/observable-lifecycle',
        },
        {
          text: 'Cold Observable e Hot Observable',
          link: '/pt/guide/observables/cold-and-hot-observables',
        },
      ],
    },
    {
      text: '3. Creation Functions',
      link: '/pt/guide/creation-functions/',
      items: [
        {
          text: 'Criação Básica',
          link: '/pt/guide/creation-functions/basic/',
          collapsed: true,
          items: [
            { text: 'of', link: '/pt/guide/creation-functions/basic/of' },
            { text: 'from', link: '/pt/guide/creation-functions/basic/from' },
            { text: 'fromEvent', link: '/pt/guide/creation-functions/basic/fromEvent' },
            { text: 'interval', link: '/pt/guide/creation-functions/basic/interval' },
            { text: 'timer', link: '/pt/guide/creation-functions/basic/timer' },
          ],
        },
        {
          text: 'Geração de Loop',
          link: '/pt/guide/creation-functions/loop/',
          collapsed: true,
          items: [
            { text: 'range', link: '/pt/guide/creation-functions/loop/range' },
            { text: 'generate', link: '/pt/guide/creation-functions/loop/generate' },
          ],
        },
        {
          text: 'Comunicação HTTP',
          link: '/pt/guide/creation-functions/http-communication/',
          collapsed: true,
          items: [
            { text: 'ajax', link: '/pt/guide/creation-functions/http-communication/ajax' },
            { text: 'fromFetch', link: '/pt/guide/creation-functions/http-communication/fromFetch' },
          ],
        },
        {
          text: 'Combinação',
          link: '/pt/guide/creation-functions/combination/',
          collapsed: true,
          items: [
            { text: 'concat', link: '/pt/guide/creation-functions/combination/concat' },
            { text: 'merge', link: '/pt/guide/creation-functions/combination/merge' },
            { text: 'combineLatest', link: '/pt/guide/creation-functions/combination/combineLatest' },
            { text: 'zip', link: '/pt/guide/creation-functions/combination/zip' },
            { text: 'forkJoin', link: '/pt/guide/creation-functions/combination/forkJoin' },
            { text: 'forkJoin vs combineLatest', link: '/pt/guide/creation-functions/combination/forkJoin-vs-combineLatest' },
          ],
        },
        {
          text: 'Seleção/Partição',
          link: '/pt/guide/creation-functions/selection/',
          collapsed: true,
          items: [
            { text: 'race', link: '/pt/guide/creation-functions/selection/race' },
            { text: 'partition', link: '/pt/guide/creation-functions/selection/partition' },
          ],
        },
        {
          text: 'Ramificação Condicional',
          link: '/pt/guide/creation-functions/conditional/',
          collapsed: true,
          items: [
            { text: 'iif', link: '/pt/guide/creation-functions/conditional/iif' },
            { text: 'defer', link: '/pt/guide/creation-functions/conditional/defer' },
          ],
        },
        {
          text: 'Controle',
          link: '/pt/guide/creation-functions/control/',
          collapsed: true,
          items: [
            { text: 'scheduled', link: '/pt/guide/creation-functions/control/scheduled' },
            { text: 'using', link: '/pt/guide/creation-functions/control/using' },
          ],
        },
      ],
    },
    {
      text: '4. Entendendo Operadores',
      link: '/pt/guide/operators/',
      items: [
        { text: 'Conceitos de Pipeline', link: '/pt/guide/operators/pipeline' },
        {
          text: 'Operadores de Transformação',
          link: '/pt/guide/operators/transformation/',
          collapsed: true,
          items: [
            { text: 'map', link: '/pt/guide/operators/transformation/map' },
            { text: 'scan', link: '/pt/guide/operators/transformation/scan' },
            { text: 'mergeScan', link: '/pt/guide/operators/transformation/mergeScan' },
            { text: 'reduce', link: '/pt/guide/operators/transformation/reduce' },
            { text: 'pairwise', link: '/pt/guide/operators/transformation/pairwise' },
            { text: 'groupBy', link: '/pt/guide/operators/transformation/groupBy' },
            {
              text: 'mergeMap',
              link: '/pt/guide/operators/transformation/mergeMap',
            },
            {
              text: 'switchMap',
              link: '/pt/guide/operators/transformation/switchMap',
            },
            {
              text: 'concatMap',
              link: '/pt/guide/operators/transformation/concatMap',
            },
            {
              text: 'exhaustMap',
              link: '/pt/guide/operators/transformation/exhaustMap',
            },
            {
              text: 'expand',
              link: '/pt/guide/operators/transformation/expand',
            },
            {
              text: 'buffer',
              link: '/pt/guide/operators/transformation/buffer',
            },
            {
              text: 'bufferTime',
              link: '/pt/guide/operators/transformation/bufferTime',
            },
            {
              text: 'bufferCount',
              link: '/pt/guide/operators/transformation/bufferCount',
            },
            {
              text: 'bufferWhen',
              link: '/pt/guide/operators/transformation/bufferWhen',
            },
            {
              text: 'bufferToggle',
              link: '/pt/guide/operators/transformation/bufferToggle',
            },
            {
              text: 'windowTime',
              link: '/pt/guide/operators/transformation/windowTime',
            },
            {
              text: 'window',
              link: '/pt/guide/operators/transformation/window',
            },
            {
              text: 'windowCount',
              link: '/pt/guide/operators/transformation/windowCount',
            },
            {
              text: 'windowToggle',
              link: '/pt/guide/operators/transformation/windowToggle',
            },
            {
              text: 'windowWhen',
              link: '/pt/guide/operators/transformation/windowWhen',
            },
            {
              text: 'Casos de Uso Práticos',
              link: '/pt/guide/operators/transformation/practical-use-cases.md',
            },
          ],
        },
        {
          text: 'Operadores de Filtragem',
          link: '/pt/guide/operators/filtering/',
          collapsed: true,
          items: [
            { text: 'filter', link: '/pt/guide/operators/filtering/filter' },
            { text: 'take', link: '/pt/guide/operators/filtering/take' },
            { text: 'takeLast', link: '/pt/guide/operators/filtering/takeLast' },
            { text: 'takeWhile', link: '/pt/guide/operators/filtering/takeWhile' },
            { text: 'skip', link: '/pt/guide/operators/filtering/skip' },
            { text: 'skipLast', link: '/pt/guide/operators/filtering/skipLast' },
            { text: 'skipWhile', link: '/pt/guide/operators/filtering/skipWhile' },
            { text: 'skipUntil', link: '/pt/guide/operators/filtering/skipUntil' },
            { text: 'first', link: '/pt/guide/operators/filtering/first' },
            { text: 'last', link: '/pt/guide/operators/filtering/last' },
            { text: 'elementAt', link: '/pt/guide/operators/filtering/elementAt' },
            { text: 'find', link: '/pt/guide/operators/filtering/find' },
            { text: 'findIndex', link: '/pt/guide/operators/filtering/findIndex' },
            {
              text: 'debounceTime',
              link: '/pt/guide/operators/filtering/debounceTime',
            },
            {
              text: 'throttleTime',
              link: '/pt/guide/operators/filtering/throttleTime',
            },
            {
              text: 'auditTime',
              link: '/pt/guide/operators/filtering/auditTime',
            },
            { text: 'audit', link: '/pt/guide/operators/filtering/audit' },
            { text: 'sampleTime', link: '/pt/guide/operators/filtering/sampleTime' },
            { text: 'ignoreElements', link: '/pt/guide/operators/filtering/ignoreElements' },
            {
              text: 'distinct',
              link: '/pt/guide/operators/filtering/distinct',
            },
            {
              text: 'distinctUntilChanged',
              link: '/pt/guide/operators/filtering/distinctUntilChanged',
            },
            {
              text: 'distinctUntilKeyChanged',
              link: '/pt/guide/operators/filtering/distinctUntilKeyChanged',
            },
            {
              text: 'Casos de Uso Práticos',
              link: '/pt/guide/operators/filtering/practical-use-cases.md',
            },
          ],
        },
        {
          text: 'Operadores de Combinação (Pipeable)',
          link: '/pt/guide/operators/combination/',
          collapsed: true,
          items: [
            {
              text: 'concatWith',
              link: '/pt/guide/operators/combination/concatWith',
            },
            {
              text: 'mergeWith',
              link: '/pt/guide/operators/combination/mergeWith',
            },
            {
              text: 'combineLatestWith',
              link: '/pt/guide/operators/combination/combineLatestWith',
            },
            {
              text: 'zipWith',
              link: '/pt/guide/operators/combination/zipWith',
            },
            {
              text: 'raceWith',
              link: '/pt/guide/operators/combination/raceWith',
            },
            {
              text: 'withLatestFrom',
              link: '/pt/guide/operators/combination/withLatestFrom',
            },
            {
              text: 'mergeAll',
              link: '/pt/guide/operators/combination/mergeAll',
            },
            {
              text: 'concatAll',
              link: '/pt/guide/operators/combination/concatAll',
            },
            {
              text: 'switchAll',
              link: '/pt/guide/operators/combination/switchAll',
            },
            {
              text: 'exhaustAll',
              link: '/pt/guide/operators/combination/exhaustAll',
            },
            {
              text: 'combineLatestAll',
              link: '/pt/guide/operators/combination/combineLatestAll',
            },
            {
              text: 'zipAll',
              link: '/pt/guide/operators/combination/zipAll',
            },
            {
              text: 'Casos de Uso Práticos',
              link: '/pt/guide/operators/combination/practical-use-cases.md',
            },
          ],
        },
        {
          text: 'Operadores Utilitários',
          link: '/pt/guide/operators/utility/',
          collapsed: true,
          items: [
            { text: 'tap', link: '/pt/guide/operators/utility/tap' },
            { text: 'delay', link: '/pt/guide/operators/utility/delay' },
            { text: 'delayWhen', link: '/pt/guide/operators/utility/delayWhen' },
            { text: 'timeout', link: '/pt/guide/operators/utility/timeout' },
            { text: 'takeUntil', link: '/pt/guide/operators/utility/takeUntil' },
            { text: 'finalize', link: '/pt/guide/operators/utility/finalize' },
            { text: 'repeat', link: '/pt/guide/operators/utility/repeat' },
            { text: 'retry', link: '/pt/guide/operators/utility/retry' },
            { text: 'startWith', link: '/pt/guide/operators/utility/startWith' },
            { text: 'toArray', link: '/pt/guide/operators/utility/toArray' },
            { text: 'materialize', link: '/pt/guide/operators/utility/materialize' },
            { text: 'dematerialize', link: '/pt/guide/operators/utility/dematerialize' },
            { text: 'observeOn', link: '/pt/guide/operators/utility/observeOn' },
            { text: 'subscribeOn', link: '/pt/guide/operators/utility/subscribeOn' },
            { text: 'timestamp', link: '/pt/guide/operators/utility/timestamp' },
            {
              text: 'Casos de Uso Práticos',
              link: '/pt/guide/operators/utility/practical-use-cases.md',
            },
          ],
        },
        {
          text: 'Operadores Condicionais',
          link: '/pt/guide/operators/conditional/',
          collapsed: true,
          items: [
            {
              text: 'defaultIfEmpty',
              link: '/pt/guide/operators/conditional/defaultIfEmpty',
            },
            { text: 'every', link: '/pt/guide/operators/conditional/every' },
            { text: 'isEmpty', link: '/pt/guide/operators/conditional/isEmpty' },
            {
              text: 'Casos de Uso Práticos',
              link: '/pt/guide/operators/conditional/practical-use-cases.md',
            },
          ],
        },
        {
          text: 'Multicasting',
          link: '/pt/guide/operators/multicasting/',
          collapsed: true,
          items: [
            { text: 'share', link: '/pt/guide/operators/multicasting/share' },
            { text: 'shareReplay', link: '/pt/guide/operators/multicasting/shareReplay' },
          ],
        },
      ],
    },
    {
      text: '5. Subject e Multicast',
      items: [
        { text: 'O que é Subject?', link: '/pt/guide/subjects/what-is-subject' },
        { text: 'Tipos de Subject', link: '/pt/guide/subjects/types-of-subject' },
        {
          text: 'Como o Multicasting Funciona',
          link: '/pt/guide/subjects/multicasting',
        },
        { text: 'Casos de Uso de Subject', link: '/pt/guide/subjects/use-cases' },
      ],
    },
    {
      text: '6. Tratamento de Erros',
      items: [
        { text: 'Estratégias de Tratamento de Erros', link: '/pt/guide/error-handling/strategies' },
        {
          text: 'Dois Locais para Tratamento de Erros',
          link: '/pt/guide/error-handling/error-handling-locations',
        },
        {
          text: 'Integração try-catch e RxJS',
          link: '/pt/guide/error-handling/try-catch-integration',
        },
        {
          text: 'retry e catchError',
          link: '/pt/guide/error-handling/retry-catch',
        },
        {
          text: 'finalize e complete',
          link: '/pt/guide/error-handling/finalize',
        },
      ],
    },
    {
      text: '7. Uso de Scheduler',
      items: [
        { text: 'Controle de Processamento Assíncrono', link: '/pt/guide/schedulers/async-control' },
        {
          text: 'Tipos de Schedulers e Como Usar',
          link: '/pt/guide/schedulers/types',
        },
        {
          text: 'Suplemento: Conhecimento Básico de Tasks e Schedulers',
          link: '/pt/guide/schedulers/task-and-scheduler-basics',
        },
      ],
    },
    {
      text: '8. Técnicas de Debug para RxJS',
      items: [
        { text: 'Visão Geral das Técnicas de Debug', link: '/pt/guide/debugging/' },
        { text: 'Cenários Comuns de Debug', link: '/pt/guide/debugging/common-scenarios' },
        { text: 'Ferramentas de Debug Personalizadas', link: '/pt/guide/debugging/custom-tools' },
        { text: 'Debug de Performance', link: '/pt/guide/debugging/performance' },
      ],
    },
    {
      text: '9. Métodos de Teste',
      items: [
        { text: 'Testes Unitários RxJS', link: '/pt/guide/testing/unit-tests' },
        {
          text: 'Uso do TestScheduler',
          link: '/pt/guide/testing/test-scheduler',
        },
        { text: 'Marble Testing', link: '/pt/guide/testing/marble-testing' },
      ],
    },
    {
      text: '10. Coleção de Anti-Padrões RxJS',
      items: [
        { text: 'O que são Anti-Padrões?', link: '/pt/guide/anti-patterns/' },
        { text: 'Erros Comuns e Como Lidar', link: '/pt/guide/anti-patterns/common-mistakes' },
        { text: 'Proliferação de Flags de Gerenciamento de Estado', link: '/pt/guide/anti-patterns/flag-management' },
        { text: 'Inferno de if Aninhados no subscribe', link: '/pt/guide/anti-patterns/subscribe-if-hell' },
        { text: 'Mistura de Promise e Observable', link: '/pt/guide/anti-patterns/promise-observable-mixing' },
        { text: 'Inferno de One-Liner e Sintaxe de Separação de Fases', link: '/pt/guide/anti-patterns/one-liner-hell' },
        { text: 'Checklist para Evitar Anti-Padrões', link: '/pt/guide/anti-patterns/checklist' },
      ],
    },
    {
      text: '11. Superando Dificuldades do RxJS',
      items: [
        { text: 'Por que o RxJS é tão Difícil?', link: '/pt/guide/overcoming-difficulties/' },
        { text: 'Barreira de Compreensão Conceitual', link: '/pt/guide/overcoming-difficulties/conceptual-understanding' },
        { text: 'Barreira de Gerenciamento de Ciclo de Vida', link: '/pt/guide/overcoming-difficulties/lifecycle-management' },
        { text: 'Confusão na Seleção de Operadores', link: '/pt/guide/overcoming-difficulties/operator-selection' },
        { text: 'Entendendo Timing e Ordem', link: '/pt/guide/overcoming-difficulties/timing-and-order' },
        { text: 'Dificuldade no Gerenciamento de Estado', link: '/pt/guide/overcoming-difficulties/state-and-sharing' },
        { text: 'Combinações de Múltiplos Streams', link: '/pt/guide/overcoming-difficulties/stream-combination' },
        { text: 'Barreiras de Debug', link: '/pt/guide/overcoming-difficulties/debugging-guide' },
      ],
    },
    {
      text: '12. Integração Avançada TypeScript e RxJS',
      // items: [
      //   {
      //     text: 'Integração Básica TypeScript e RxJS',
      //     link: '/pt/guide/typescript-advanced/type-safety',
      //   },
      //   {
      //     text: 'Uso de Generics',
      //     link: '/pt/guide/typescript-advanced/generics',
      //   },
      //   {
      //     text: 'Operadores Personalizados e Definições de Tipo',
      //     link: '/pt/guide/typescript-advanced/custom-operators',
      //   },
      //   {
      //     text: 'Uso de Tipos Condicionais e Mapped',
      //     link: '/pt/guide/typescript-advanced/conditional-types',
      //   },
      // ],
    },
    {
      text: '13. Padrões Práticos',
      items: [
        { text: 'Visão Geral dos Padrões Práticos', link: '/pt/guide/practical-patterns/' },
        { text: 'Tratamento de Eventos de UI', link: '/pt/guide/practical-patterns/ui-events' },
        { text: 'Chamadas de API', link: '/pt/guide/practical-patterns/api-calls' },
        { text: 'Processamento de Formulários', link: '/pt/guide/practical-patterns/form-handling' },
        { text: 'Padrões Avançados de Formulários', link: '/pt/guide/practical-patterns/advanced-form-patterns' },
        { text: 'Processamento de Dados em Tempo Real', link: '/pt/guide/practical-patterns/real-time-data' },
        { text: 'Estratégias de Cache', link: '/pt/guide/practical-patterns/caching-strategies' },
        { text: 'Práticas de Tratamento de Erros', link: '/pt/guide/practical-patterns/error-handling-patterns' },
        { text: 'Ramificações Condicionais no subscribe', link: '/pt/guide/practical-patterns/subscribe-branching' },
      ],
    },
    {
      text: '14. Otimização de Performance',
      // items: [
      //   {
      //     text: 'Gerenciamento Correto de Subscriptions',
      //     link: '/pt/guide/performance/subscription-management',
      //   },
      //   {
      //     text: 'Seleção Eficiente de Operadores',
      //     link: '/pt/guide/performance/operator-selection',
      //   },
      //   {
      //     text: 'Padrões de Design de Stream',
      //     link: '/pt/guide/performance/stream-design',
      //   },
      // ],
    },
    {
      text: '15. Integração com Frameworks',
      // items: [
      //   { text: 'Integração com Angular', link: '/pt/guide/frameworks/angular' },
      //   { text: 'Integração com React', link: '/pt/guide/frameworks/react' },
      //   { text: 'Integração com Vue', link: '/pt/guide/frameworks/vue' },
      //   { text: 'Integração com Web API', link: '/pt/guide/frameworks/web-api' },
      // ],
    },
    {
      text: 'Apêndice',
      items: [
        { text: 'Visão Geral do Apêndice', link: '/pt/guide/appendix/' },
        { text: 'Desenvolvimento Embarcado e Programação Reativa', link: '/pt/guide/appendix/embedded-reactive-programming' },
        { text: 'Métodos Reativos além do ReactiveX', link: '/pt/guide/appendix/reactive-patterns-beyond-rxjs' },
        { text: 'Visão Geral das Arquiteturas Reativas', link: '/pt/guide/appendix/reactive-architecture-map' },
        { text: 'Programação Reativa Reconsiderada', link: '/pt/guide/appendix/reactive-programming-reconsidered' },
        { text: 'Ecossistema RxJS e Reactive Streams', link: '/pt/guide/appendix/rxjs-and-reactive-streams-ecosystem' },
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
    message: 'Lançado sob a licença CC-BY-4.0.',
    copyright: 'Copyright © 2025 shuji-bonji',
  },
};
