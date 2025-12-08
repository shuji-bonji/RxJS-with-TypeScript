---
description: "Una guía de aprendizaje para estudiar sistemáticamente RxJS en un entorno TypeScript. Proporciona explicaciones prácticas paso a paso que cubren todo, desde los fundamentos de Observable hasta Subjects, varios operadores, manejo de errores, programadores y técnicas de prueba."
---

# Guía

Esta guía te ayuda a aprender sistemáticamente RxJS en un entorno TypeScript.
Progresando a través de las siguientes secciones en orden, puedes obtener una comprensión estructurada de RxJS desde los fundamentos hasta los conceptos avanzados.

## Índice

### 1. Introducción a RxJS
- [Primeros pasos](/es/guide/introduction)
- [Configuración del entorno de aprendizaje](/es/guide/starter-kid)
- [¿Qué es RxJS?](/es/guide/basics/what-is-rxjs)
- [¿Qué es un Stream?](/es/guide/basics/what-is-a-stream)
- [Promise vs. RxJS](/es/guide/basics/promise-vs-rxjs)

### 2. Fundamentos de los Observables
- [¿Qué es un Observable?](/es/guide/observables/what-is-observable)
- [Cómo crear un Observable](/es/guide/observables/creation)
- [Eventos en Streaming](/es/guide/observables/events)
- [Lista de Eventos](/es/guide/observables/events-list)
- [Observer vs Subscriber](/es/guide/observables/observer-vs-subscriber)
- [Ciclo de Vida de un Observable](/es/guide/observables/observable-lifecycle)
- [Observables Fríos y Observables Calientes](/es/guide/observables/cold-and-hot-observables)

### 3. Funciones de creación
- [¿Qué son las funciones de creación?](/es/guide/creation-functions/)
- [Funciones básicas de creación](/es/guide/creation-functions/basic/) - of, from, fromEvent, interval, timer
- [Funciones de generación de bucles](/es/guide/creation-functions/loop/) - range, generate
- [Funciones de comunicación HTTP](/es/guide/creation-functions/http-communication/) - ajax, fromFetch
- [Funciones de combinación](/es/guide/creation-functions/combination/) - concat, merge, combineLatest, zip, forkJoin
- [Funciones de Selección y Partición](/es/guide/creation-functions/selection/) - race, partition
- [Funciones condicionales](/es/guide/creation-functions/conditional/) - iif, defer
- [Funciones de control](/es/guide/creation-functions/control/) - scheduled, using

### 4. Comprensión de los operadores
- [Visión General de los Operadores](/es/guide/operators/)
- [Conceptos de Pipeline](/es/guide/operators/pipeline)
- [Operadores de transformación](/es/guide/operators/transformation/) - map, scan, mergeMap, switchMap, etc.
- [Operadores de filtrado](/es/guide/operators/filtering/) - filter, take, debounceTime, throttleTime, etc.
- [Operadores de combinación](/es/guide/operators/combination/) - withLatestFrom, mergeWith, etc.
- [Operadores de utilidad](/es/guide/operators/utility/) - tap, delay, retry, finalize, takeUntil, etc.
- [Multicasting](/es/guide/operators/multicasting/) - share, shareReplay, etc.

### 5. Subjects y Multicasting
- [¿Qué es un Subject?](/es/guide/subjects/what-is-subject)
- [Tipos de Subject](/es/guide/subjects/types-of-subject)
- [Funcionamiento de la multidifusión](/es/guide/subjects/multicasting)
- [Casos de uso de Subject](/es/guide/subjects/use-cases)

### 6. Tratamiento de errores
- [Estrategias de tratamiento de errores](/es/guide/error-handling/strategies)
- [Dos ubicaciones para el manejo de errores](/es/guide/error-handling/error-handling-locations)
- [Integración de try-catch con RxJS](/es/guide/error-handling/try-catch-integration)
- [retry y catchError](/es/guide/error-handling/retry-catch)
- [finalize y complete](/es/guide/error-handling/finalize)

### 7. Utilización de Schedulers
- [Control del procesamiento asíncrono](/es/guide/schedulers/async-control)
- [Tipos de Schedulers y Uso](/es/guide/schedulers/types)
- [Fundamentos de Tareas y Schedulers](/es/guide/schedulers/task-and-scheduler-basics)

### 8. Técnicas de depuración de RxJS
- [Técnicas de depuración](/es/guide/debugging/)
- [Escenarios comunes de depuración](/es/guide/debugging/common-scenarios)
- [Herramientas de depuración personalizadas](/es/guide/debugging/custom-tools)
- [Depuración del rendimiento](/es/guide/debugging/performance)

### 9. Técnicas de prueba
- [Pruebas unitarias de RxJS](/es/guide/testing/unit-tests)
- [Uso de TestScheduler](/es/guide/testing/test-scheduler)
- [Marble Testing](/es/guide/testing/marble-testing)

### 10. Colección de Anti-Patrones RxJS
- [¿Qué son los Anti-Patrones?](/es/guide/anti-patterns/)
- [Errores comunes y soluciones](/es/guide/anti-patterns/common-mistakes)
- [Proliferación de flags](/es/guide/anti-patterns/flag-management)
- [Sentencias if anidadas en subscribe](/es/guide/anti-patterns/subscribe-if-hell)
- [Mezcla de promesas y observables](/es/guide/anti-patterns/promise-observable-mixing)
- [Infierno de una línea](/es/guide/anti-patterns/one-liner-hell)
- [Lista de comprobación para evitar antipatrones](/es/guide/anti-patterns/checklist)

### 11. Superar las dificultades de RxJS
- [Por qué RxJS es difícil](/es/guide/overcoming-difficulties/)
- [El obstáculo de la comprensión conceptual](/es/guide/overcoming-difficulties/conceptual-understanding)
- [El obstáculo de la gestión del ciclo de vida](/es/guide/overcoming-difficulties/lifecycle-management)
- [Dilemas de selección de operadores](/es/guide/overcoming-difficulties/operator-selection)
- [Comprensión de la sincronización y el orden](/es/guide/overcoming-difficulties/timing-and-order)
- [Dificultad con la gestión de estados](/es/guide/overcoming-difficulties/state-and-sharing)
- [Combinación de múltiples flujos](/es/guide/overcoming-difficulties/stream-combination)
- [Desafíos de depuración](/es/guide/overcoming-difficulties/debugging-guide)

### 13. Colección de Patrones Prácticos
- [Visión General de los Patrones Prácticos](/es/guide/practical-patterns/)
- [Gestión de eventos de interfaz de usuario](/es/guide/practical-patterns/ui-events)
- [Llamadas a la API](/es/guide/practical-patterns/api-calls)
- [Gestión de formularios](/es/guide/practical-patterns/form-handling)
- [Patrones de formularios avanzados](/es/guide/practical-patterns/advanced-form-patterns)
- [Procesamiento de datos en tiempo real](/es/guide/practical-patterns/real-time-data)
- [Estrategias de almacenamiento en caché](/es/guide/practical-patterns/caching-strategies)
- [Patrones de gestión de errores](/es/guide/practical-patterns/error-handling-patterns)
- [Bifurcación condicional en las suscripciones](/es/guide/practical-patterns/subscribe-branching)

### Apéndice
- [Resumen del apéndice](/es/guide/appendix/)
- [Desarrollo Integrado y Programación Reactiva](/es/guide/appendix/embedded-reactive-programming)
- [Patrones Reactivos más allá de RxJS](/es/guide/appendix/reactive-patterns-beyond-rxjs)
- [Mapa de Arquitectura Reactiva](/es/guide/appendix/reactive-architecture-map)
- [Programación reactiva reconsiderada](/es/guide/appendix/reactive-programming-reconsidered)
- [Ecosistema RxJS y Reactive Streams](/es/guide/appendix/rxjs-and-reactive-streams-ecosystem)

---

> [!NOTE]
> Esta guía está estructurada para profundizar tu comprensión de RxJS de una manera sistemática y paso a paso. Siéntete libre de hacer referencia a cualquier sección según sea necesario.
