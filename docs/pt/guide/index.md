---
Description: Um guia de aprendizado para estudar sistematicamente RxJS em um ambiente TypeScript. Fornece explicações práticas passo a passo cobrindo tudo desde os fundamentos de Observable até Subjects, vários operadores, tratamento de erros, schedulers e técnicas de teste. Cada seção pode ser referenciada de forma independente.
---

# Guia

Este guia ajuda você a aprender sistematicamente RxJS em um ambiente TypeScript.
Ao progredir pelas seções abaixo em ordem, você pode obter uma compreensão estruturada de RxJS desde os fundamentos até conceitos avançados.

## Índice

### 1. Introdução ao RxJS
- [Primeiros Passos](/pt/guide/introduction)
- [Configurando Seu Ambiente de Aprendizado](/pt/guide/starter-kid.md)
- [O que é RxJS?](/pt/guide/basics/what-is-rxjs)
- [O que é um Stream?](/pt/guide/basics/what-is-a-stream)
- [Promise vs. RxJS](/pt/guide/basics/promise-vs-rxjs)

### 2. Fundamentos de Observable
- [O que é um Observable?](/pt/guide/observables/what-is-observable)
- [Como Criar um Observable](/pt/guide/observables/creation)
- [Streaming de Eventos](/pt/guide/observables/events)
- [Eventos Não Utilizáveis com fromEvent](/pt/guide/observables/events#cannot-used-fromEvent)
- [Lista de Eventos](/pt/guide/observables/events-list)
- [Ciclo de Vida do Observable](/pt/guide/observables/observable-lifecycle)
- [Cold Observables e Hot Observables](/pt/guide/observables/cold-and-hot-observables)

### 3. Creation Functions
- [O que são Creation Functions?](/pt/guide/creation-functions/)
- [Creation Functions Básicas](/pt/guide/creation-functions/basic/) - of, from, fromEvent, interval, timer
- [Funções de Geração em Loop](/pt/guide/creation-functions/loop/) - range, generate
- [Funções de Comunicação HTTP](/pt/guide/creation-functions/http-communication/) - ajax, fromFetch
- [Funções de Combinação](/pt/guide/creation-functions/combination/) - concat, merge, combineLatest, zip, forkJoin
- [Funções de Seleção e Partição](/pt/guide/creation-functions/selection/) - race, partition
- [Funções Condicionais](/pt/guide/creation-functions/conditional/) - iif, defer
- [Funções de Controle](/pt/guide/creation-functions/control/) - scheduled, using

### 4. Entendendo os Operadores
- [Visão Geral dos Operadores](/pt/guide/operators/)
- [Conceitos de Pipeline](/pt/guide/operators/pipeline)
- [Operadores de Transformação](/pt/guide/operators/transformation/) - map, scan, mergeMap, switchMap, relacionados a buffer, relacionados a window, etc.
- [Operadores de Filtragem](/pt/guide/operators/filtering/) - filter, take, debounceTime, throttleTime, distinct, etc.
- [Operadores de Combinação](/pt/guide/operators/combination/) - concatWith, mergeWith, withLatestFrom, operadores *All, etc.
- [Operadores Utilitários](/pt/guide/operators/utility/) - tap, delay, retry, finalize, takeUntil, etc.
- [Operadores Condicionais](/pt/guide/operators/conditional/) - defaultIfEmpty, every, isEmpty, etc.
- [Multicasting](/pt/guide/operators/multicasting/) - share, shareReplay, etc.

### 5. Subjects e Multicasting
- [O que é um Subject?](/pt/guide/subjects/what-is-subject)
- [Tipos de Subject](/pt/guide/subjects/types-of-subject)
- [Como Funciona o Multicasting](/pt/guide/subjects/multicasting)
- [Casos de Uso de Subject](/pt/guide/subjects/use-cases)

### 6. Tratamento de Erros
- [Estratégias de Tratamento de Erros](/pt/guide/error-handling/strategies)
- [Dois Locais para Tratamento de Erros](/pt/guide/error-handling/error-handling-locations)
- [Integrando try-catch com RxJS](/pt/guide/error-handling/try-catch-integration)
- [retry e catchError](/pt/guide/error-handling/retry-catch)
- [finalize e complete](/pt/guide/error-handling/finalize)

### 7. Utilizando Schedulers
- [Controlando Processamento Assíncrono](/pt/guide/schedulers/async-control)
- [Tipos e Uso de Scheduler](/pt/guide/schedulers/types)
- [Suplemento: Fundamentos de Task e Scheduler](/pt/guide/schedulers/task-and-scheduler-basics)

### 8. Técnicas de Depuração RxJS
- [Visão Geral das Técnicas de Depuração](/pt/guide/debugging/)
- [Cenários Comuns de Depuração](/pt/guide/debugging/common-scenarios)
- [Ferramentas de Depuração Personalizadas](/pt/guide/debugging/custom-tools)
- [Depuração de Performance](/pt/guide/debugging/performance)

### 9. Técnicas de Teste
- [Testes Unitários RxJS](/pt/guide/testing/unit-tests)
- [Usando TestScheduler](/pt/guide/testing/test-scheduler)
- [Marble Testing](/pt/guide/testing/marble-testing)

### 10. Coleção de Anti-Padrões RxJS
- [O que São Anti-Padrões?](/pt/guide/anti-patterns/)
- [Erros Comuns e Soluções](/pt/guide/anti-patterns/common-mistakes)
- [Declarações if Aninhadas em subscribe](/pt/guide/anti-patterns/subscribe-if-hell)
- [Misturando Promises e Observables](/pt/guide/anti-patterns/promise-observable-mixing)
- [Inferno de Uma Linha e Separação de Responsabilidades](/pt/guide/anti-patterns/one-liner-hell)
- [Checklist de Prevenção de Anti-Padrões](/pt/guide/anti-patterns/checklist)

### 11. Superando Dificuldades com RxJS
- [Por que RxJS é Difícil](/pt/guide/overcoming-difficulties/)
- [Barreira de Compreensão Conceitual](/pt/guide/overcoming-difficulties/conceptual-understanding)
- [O Obstáculo do Gerenciamento de Ciclo de Vida](/pt/guide/overcoming-difficulties/lifecycle-management)
- [Dilemas de Seleção de Operadores](/pt/guide/overcoming-difficulties/operator-selection)
- [Entendendo Timing e Ordem](/pt/guide/overcoming-difficulties/timing-and-order)
- [Dificuldade com Gerenciamento de Estado](/pt/guide/overcoming-difficulties/state-and-sharing)
- [Combinando Múltiplos Streams](/pt/guide/overcoming-difficulties/stream-combination)
- [Desafios de Depuração](/pt/guide/overcoming-difficulties/debugging-guide)

### 13. Coleção de Padrões Práticos
- [Visão Geral dos Padrões Práticos](/pt/guide/practical-patterns/)
- [Tratamento de Eventos de UI](/pt/guide/practical-patterns/ui-events) - Cliques, rolagens, arrastar e soltar, etc.
- [Chamadas de API](/pt/guide/practical-patterns/api-calls) - Comunicação HTTP, processamento paralelo/serial, tratamento de erros
- [Tratamento de Formulários](/pt/guide/practical-patterns/form-handling) - Validação em tempo real, salvamento automático, coordenação de múltiplos campos
- [Processamento de Dados em Tempo Real](/pt/guide/practical-patterns/real-time-data) - WebSocket, SSE, Polling, gerenciamento de conexão
- [Estratégias de Cache](/pt/guide/practical-patterns/caching-strategies) - Cache de dados, TTL, invalidação, suporte offline
- [Padrões de Tratamento de Erros](/pt/guide/practical-patterns/error-handling-patterns) - Erros em chamadas de API, estratégias de retry, tratamento global de erros
- [Ramificação Condicional em Subscriptions](/pt/guide/practical-patterns/subscribe-branching) - Evitando ramificação dentro de subscriptions, métodos de ramificação dentro de pipelines

### Apêndice
- [Visão Geral do Apêndice](/pt/guide/appendix/)
- [Desenvolvimento Embarcado e Programação Reativa](/pt/guide/appendix/embedded-reactive-programming)
- [Padrões Reativos Além do RxJS](/pt/guide/appendix/reactive-patterns-beyond-rxjs)
- [Mapa de Arquitetura Reativa](/pt/guide/appendix/reactive-architecture-map)

---

> [!NOTE]
> Este guia é estruturado para aprofundar sua compreensão de RxJS de maneira gradual e sistemática.
> Sinta-se à vontade para consultar qualquer seção conforme necessário.
