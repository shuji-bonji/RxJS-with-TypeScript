---
description: Een leergids voor het systematisch leren van RxJS in een TypeScript-omgeving. Behandelt stapsgewijs en praktisch alles van Observable-basisprincipes tot Subject, verschillende operators, foutafhandeling, schedulers en testmethoden. Elk hoofdstuk kan onafhankelijk worden geraadpleegd.
---

# Gids

Een gids voor het systematisch leren van RxJS in een TypeScript-omgeving.
Door de volgende hoofdstukken in volgorde te doorlopen, kunt u RxJS systematisch begrijpen van de basis tot geavanceerde onderwerpen.

## Inhoudsopgave

### 1. Introductie tot RxJS
- [Inleiding](/nl/guide/introduction)
- [Ontwikkelomgeving opzetten voor leren](/nl/guide/starter-kid.md)
- [Wat is RxJS](/nl/guide/basics/what-is-rxjs)
- [Wat is een Stream?](/nl/guide/basics/what-is-a-stream)
- [Verschil tussen Promise en RxJS](/nl/guide/basics/promise-vs-rxjs)

### 2. Observable Basisprincipes
- [Wat is een Observable](/nl/guide/observables/what-is-observable)
- [Hoe maak je een Observable](/nl/guide/observables/creation)
- [Events naar Streams converteren](/nl/guide/observables/events)
- [Events die niet met fromEvent kunnen worden gebruikt](/nl/guide/observables/events#cannot-used-fromEvent)
- [Lijst van Events](/nl/guide/observables/events-list)
- [Observable Levenscyclus](/nl/guide/observables/observable-lifecycle)
- [Cold Observable en Hot Observable](/nl/guide/observables/cold-and-hot-observables)

### 3. Creation Functions
- [Wat zijn Creation Functions](/nl/guide/creation-functions/)
- [Basis creatie](/nl/guide/creation-functions/basic/) - of, from, fromEvent, interval, timer
- [Lus generatie](/nl/guide/creation-functions/loop/) - range, generate
- [HTTP communicatie](/nl/guide/creation-functions/http-communication/) - ajax, fromFetch
- [Combinatie](/nl/guide/creation-functions/combination/) - concat, merge, combineLatest, zip, forkJoin
- [Selectie & Partitie](/nl/guide/creation-functions/selection/) - race, partition
- [Voorwaardelijk](/nl/guide/creation-functions/conditional/) - iif, defer
- [Controle](/nl/guide/creation-functions/control/) - scheduled, using

### 4. Operators Begrijpen
- [Overzicht van Operators](/nl/guide/operators/)
- [Pipeline Concept](/nl/guide/operators/pipeline)
- [Transformatie Operators](/nl/guide/operators/transformation/) - map, scan, mergeMap, switchMap, buffer-serie, window-serie, etc.
- [Filtering Operators](/nl/guide/operators/filtering/) - filter, take, debounceTime, throttleTime, distinct, etc.
- [Combinatie Operators](/nl/guide/operators/combination/) - concatWith, mergeWith, withLatestFrom, *All operators, etc.
- [Utility Operators](/nl/guide/operators/utility/) - tap, delay, retry, finalize, takeUntil, etc.
- [Voorwaardelijke Operators](/nl/guide/operators/conditional/) - defaultIfEmpty, every, isEmpty, etc.
- [Multicasting](/nl/guide/operators/multicasting/) - share, shareReplay, etc.

### 5. Subject en Multicast
- [Wat is een Subject](/nl/guide/subjects/what-is-subject)
- [Typen Subjects](/nl/guide/subjects/types-of-subject)
- [Multicasting Mechanisme](/nl/guide/subjects/multicasting)
- [Subject Gebruikscases](/nl/guide/subjects/use-cases)

### 6. Foutafhandeling
- [Foutafhandelingsstrategieën](/nl/guide/error-handling/strategies)
- [Twee Locaties voor Foutafhandeling](/nl/guide/error-handling/error-handling-locations)
- [Integratie van try-catch en RxJS](/nl/guide/error-handling/try-catch-integration)
- [retry en catchError](/nl/guide/error-handling/retry-catch)
- [finalize en complete](/nl/guide/error-handling/finalize)

### 7. Gebruik van Schedulers
- [Asynchrone Verwerking Controleren](/nl/guide/schedulers/async-control)
- [Typen Schedulers en Wanneer Welke te Gebruiken](/nl/guide/schedulers/types)
- [Aanvulling: Basiskennis van Tasks en Schedulers](/nl/guide/schedulers/task-and-scheduler-basics)

### 8. RxJS Debugging Technieken
- [Overzicht van Debugging Technieken](/nl/guide/debugging/)
- [Veelvoorkomende Debugging Scenario's](/nl/guide/debugging/common-scenarios)
- [Aangepaste Debugging Tools](/nl/guide/debugging/custom-tools)
- [Performance Debugging](/nl/guide/debugging/performance)

### 9. Testmethoden
- [Unit Testing voor RxJS](/nl/guide/testing/unit-tests)
- [TestScheduler Gebruiken](/nl/guide/testing/test-scheduler)
- [Marble Testing](/nl/guide/testing/marble-testing)

### 10. RxJS Anti-patronen
- [Wat zijn Anti-patronen](/nl/guide/anti-patterns/)
- [Veelvoorkomende Fouten en Oplossingen](/nl/guide/anti-patterns/common-mistakes)
- [If-statement Nesting Hell binnen subscribe](/nl/guide/anti-patterns/subscribe-if-hell)
- [Promise en Observable Vermenging](/nl/guide/anti-patterns/promise-observable-mixing)
- [One-liner Hell en Stapsgewijze Scheidingssyntax](/nl/guide/anti-patterns/one-liner-hell)
- [Anti-patroon Vermijdings Checklist](/nl/guide/anti-patterns/checklist)

### 11. RxJS Moeilijkheden Overwinnen
- [Waarom is RxJS Moeilijk](/nl/guide/overcoming-difficulties/)
- [Conceptueel Begrip Barrière](/nl/guide/overcoming-difficulties/conceptual-understanding)
- [Levenscyclus Beheer Barrière](/nl/guide/overcoming-difficulties/lifecycle-management)
- [Operator Selectie Twijfels](/nl/guide/overcoming-difficulties/operator-selection)
- [Timing en Volgorde Begrijpen](/nl/guide/overcoming-difficulties/timing-and-order)
- [Moeilijkheden met State Management](/nl/guide/overcoming-difficulties/state-and-sharing)
- [Meerdere Streams Combineren](/nl/guide/overcoming-difficulties/stream-combination)
- [Debugging Barrière](/nl/guide/overcoming-difficulties/debugging-guide)

### 13. Praktische Patronen
- [Overzicht van Praktische Patronen](/nl/guide/practical-patterns/)
- [UI Event Verwerking](/nl/guide/practical-patterns/ui-events) - klikken, scrollen, drag & drop, etc.
- [API Aanroepen](/nl/guide/practical-patterns/api-calls) - HTTP communicatie, parallelle/seriële verwerking, foutafhandeling
- [Formulier Verwerking](/nl/guide/practical-patterns/form-handling) - realtime validatie, auto-opslaan, meerdere velden koppelen
- [Geavanceerde Formulier Patronen](/nl/guide/practical-patterns/advanced-form-patterns) - JSON Patch, grootschalige formulier auto-opslaan, Undo/Redo, collaboratief bewerken
- [Realtime Data Verwerking](/nl/guide/practical-patterns/real-time-data) - WebSocket, SSE, Polling, verbindingsbeheer
- [Cache Strategieën](/nl/guide/practical-patterns/caching-strategies) - data caching, TTL, invalidatie, offline ondersteuning
- [Praktische Foutafhandeling](/nl/guide/practical-patterns/error-handling-patterns) - API aanroepfouten, retry strategieën, globale foutafhandeling
- [Voorwaardelijke Vertakkingen binnen subscribe](/nl/guide/practical-patterns/subscribe-branching) - vertakkingen binnen subscribe vermijden, vertakkingsmethoden binnen pipeline

### Appendix
- [Appendix Overzicht](/nl/guide/appendix/)
- [Embedded Development en Reactive Programming](/nl/guide/appendix/embedded-reactive-programming)
- [Reactieve Methoden Buiten ReactiveX](/nl/guide/appendix/reactive-patterns-beyond-rxjs)
- [Reactive Architecture Compleet Overzicht](/nl/guide/appendix/reactive-architecture-map)

---

> [!NOTE]
> Deze gids is gestructureerd om uw begrip van RxJS stapsgewijs en systematisch te verdiepen.
> Raadpleeg elk hoofdstuk vrijelijk naar behoefte.
