---
description: "Descripción general de patrones prácticos de RxJS. Aprende técnicas de implementación del mundo real para eventos UI, llamadas API, manejo de formularios, datos en tiempo real, estrategias de caché, patrones de manejo de errores, ramificación de subscribe y patrones avanzados de formularios."
---

# Patrones Prácticos

Esta sección presenta patrones de implementación práctica de RxJS que se pueden utilizar inmediatamente en proyectos del mundo real. Va más allá de los ejemplos básicos para mostrar cómo construir características reales de aplicaciones con RxJS.

## Por qué son importantes los patrones prácticos

Aprender operadores y sintaxis de RxJS es solo el comienzo. La verdadera habilidad radica en saber **qué patrones aplicar en qué situaciones**. Esta sección cierra esa brecha proporcionando soluciones probadas en batalla para desafíos comunes de desarrollo.

### Qué aprenderás

Cada artículo de patrón incluye:

1. **Escenarios del mundo real** - Casos de uso concretos con contexto empresarial
2. **Ejemplos de código ejecutables** - Todo el código es ejecutable inmediatamente (creación dinámica de DOM, sin configuración HTML)
3. **Implementaciones paso a paso** - Del simple al avanzado con explicaciones claras
4. **Mejores prácticas** - Manejo de memoria, seguridad de tipos y consideraciones de rendimiento
5. **Pruebas** - Estrategias de prueba para cada patrón
6. **Secciones relacionadas** - Enlaces a capítulos fundamentales y guías avanzadas

## Patrones Disponibles

### 1. Patrones de Eventos UI

**[Patrones de Eventos UI](./ui-events.md)**

Aprende a manejar eventos de navegador con RxJS:
- Debouncing y throttling de búsqueda en autocompletado
- Manejo de eventos de arrastrar y soltar
- Sincronización multi-touch y gestos
- Detección y cancelación de doble clic
- Patrón Observable para componentes Web

**Cuándo usar:** Aplicaciones con interacción intensiva del usuario (búsqueda, formularios, editores de arrastrar y soltar)

---

### 2. Patrones de Llamadas API

**[Patrones de Llamadas API](./api-calls.md)**

Domina los patrones de comunicación API:
- Cancelación de solicitudes con `switchMap`
- Solicitudes secuenciales con `concatMap`
- Solicitudes paralelas con `forkJoin`
- Paginación infinita y carga bajo demanda
- Polling con control de inicio/detención
- Reintentos con backoff exponencial

**Cuándo usar:** Cualquier aplicación que consuma APIs REST/HTTP

---

### 3. Patrones de Manejo de Formularios

**[Patrones de Manejo de Formularios](./form-handling.md)**

Implementa formularios complejos con RxJS:
- Validación en tiempo real (sincrónica y asincrónica)
- Validación cruzada de campos con `combineLatest`
- Autoguardado con debounce
- Estado de envío con indicadores de carga
- Confirmaciones de abandono de formulario

**Cuándo usar:** Formularios con validación compleja, formularios multi-paso, experiencias de autoguardado

---

### 4. Procesamiento de Datos en Tiempo Real

**[Procesamiento de Datos en Tiempo Real](./real-time-data.md)**

Trabaja con flujos de datos en tiempo real:
- Conexión de WebSocket con auto-reconexión
- Eventos enviados por el servidor (SSE)
- Polling con ajuste dinámico de intervalo
- Actualizaciones de transmisión en vivo con reconciliación de buffer
- Sincronización multi-fuente con fusión de datos

**Cuándo usar:** Dashboards, feeds en vivo, notificaciones, aplicaciones de chat, visualizaciones en tiempo real

---

### 5. Estrategias de Caché

**[Estrategias de Caché](./caching-strategies.md)**

Optimiza el rendimiento con patrones de caché:
- Caché en memoria con `shareReplay`
- Almacenamiento local con estrategia de caché primero
- IndexedDB para almacenamiento fuera de línea de gran volumen
- Invalidación de caché basada en tiempo y eventos
- Refrescamiento de caché en segundo plano con datos obsoletos mientras se revalida

**Cuándo usar:** Aplicaciones con llamadas API frecuentes, aplicaciones PWA fuera de línea, optimización de rendimiento

---

### 6. Patrones de Manejo de Errores en la Práctica

**[Patrones de Manejo de Errores en la Práctica](./error-handling-patterns.md)**

Maneja errores de forma elegante:
- Clasificación de errores HTTP por código de estado
- Manejo de errores de red con reintentos
- Errores de timeout con retroalimentación del usuario
- Manejo de errores globales con servicio de log
- Estrategias de reintento (inmediato, fijo, exponencial, lineal)
- Diseño de límites de error por componente

**Cuándo usar:** Todas las aplicaciones de producción (el manejo de errores es crítico para la experiencia del usuario)

---

### 7. Patrones de Ramificación de Subscribe

**[Patrones de Ramificación de Subscribe](./subscribe-branching.md)**

Evita el anti-patrón de ramificación condicional compleja dentro de `subscribe()`:
- **Patrón 1:** Ramificación con `filter` + `tap`
- **Patrón 2:** Ramificación con `partition`
- **Patrón 3:** Ramificación dinámica con `switchMap` + `iif`
- **Patrón 4:** Funcionalización + transformación con `map` (Recomendado)

Muestra cómo convertir lógica imperativa dentro de subscribe en pipelines declarativos.

**Cuándo usar:** Manejo de respuestas API, transformaciones de datos complejas, patrón ViewModel

---

### 8. Patrones Avanzados de Formularios con JSON Patch

**[Patrones Avanzados de Formularios](./advanced-form-patterns.md)**

Patrones a nivel empresarial para formularios grandes:
- Fundamentos de JSON Patch/Pointer (RFC 6902/6901)
- Autoguardado de formularios grandes (basado en diferencias)
- Implementación de Undo/Redo (parche inverso)
- Sincronización en tiempo real para edición colaborativa
- Fundamentos de Operational Transform (OT) / CRDT
- Patrones de integración WebSocket + RxJS
- Resolución de conflictos y control de versiones

**Cuándo usar:** Formularios grandes (>100 campos), edición colaborativa en tiempo real (como Google Docs), aplicaciones con funcionalidad Undo/Redo

---

## Cómo Usar Esta Sección

### Para principiantes

Comienza con patrones fundamentales:
1. **[Patrones de Eventos UI](./ui-events.md)** - Aprende manejo básico de eventos
2. **[Patrones de Llamadas API](./api-calls.md)** - Domina comunicación HTTP
3. **[Patrones de Manejo de Formularios](./form-handling.md)** - Implementa formularios interactivos

### Para usuarios intermedios

Profundiza en escenarios en tiempo real:
4. **[Procesamiento de Datos en Tiempo Real](./real-time-data.md)** - WebSocket y SSE
5. **[Estrategias de Caché](./caching-strategies.md)** - Optimización de rendimiento
6. **[Patrones de Manejo de Errores](./error-handling-patterns.md)** - Manejo robusto de errores

### Para usuarios avanzados

Implementa patrones a nivel empresarial:
7. **[Patrones de Ramificación de Subscribe](./subscribe-branching.md)** - Refactoriza código complejo
8. **[Patrones Avanzados de Formularios](./advanced-form-patterns.md)** - JSON Patch y edición colaborativa

## Conexión con Fundamentos

Cada patrón práctico se basa en conocimientos fundamentales de capítulos anteriores:

- **[Capítulo 2: Observables](../observables/what-is-observable.md)** - Entender Observables fríos vs. calientes
- **[Capítulo 3: Creation Functions](../creation-functions/index.md)** - Saber cuándo usar `fromEvent`, `interval`, `merge`
- **[Capítulo 4: Operadores](../operators/index.md)** - Dominar operadores de transformación, filtrado y combinación
- **[Capítulo 5: Subject](../subjects/index.md)** - Usar Subjects para gestión de estado
- **[Capítulo 6: Manejo de Errores](../error-handling/index.md)** - Aplicar `catchError`, `retry`, `retryWhen`
- **[Capítulo 9: Pruebas](../testing/unit-tests.md)** - Probar pipelines RxJS con TestScheduler

## Principios de Código Ejecutable

Todos los ejemplos de código en esta sección siguen estos principios:

> [!TIP] Patrones de Código Ejecutable
> 1. **Creación Dinámica de DOM** - Todo el HTML se crea con `document.createElement()` (sin HTML pre-existente requerido)
> 2. **Ejecutable en Consola del Navegador** - Copia/pega directamente en DevTools
> 3. **Trabajar con APIs Públicas** - Usar JSONPlaceholder u otras APIs REST gratuitas cuando sea posible
> 4. **Implementaciones de Mock incluidas** - Implementaciones de servidor simulado para casos donde no hay API pública disponible
> 5. **Código Completo Mostrado** - Nada de `...` o fragmentos de código incompletos

### Ejemplo: Patrón de Creación Dinámica de DOM

```typescript
// ✅ Patrón Recomendado - Creación dinámica
const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Buscar...';
document.body.appendChild(searchInput);

fromEvent(searchInput, 'input').pipe(
  debounceTime(300),
  map(event => (event.target as HTMLInputElement).value)
).subscribe(value => {
  console.log('Término de búsqueda:', value);
});

// ❌ Evitar - Requiere HTML pre-existente
// const searchInput = document.querySelector('#search-input');
```

Este enfoque asegura que cada ejemplo sea **inmediatamente ejecutable** sin ninguna configuración.

## Próximos Pasos

1. **Explorar Patrones** - Navega a través de los 8 artículos de patrones
2. **Probar Código** - Copia ejemplos en la consola de tu navegador o CodeSandbox
3. **Aplicar en Proyectos** - Adapta patrones a tus necesidades específicas
4. **Contribuir** - Comparte tus propios patrones y mejoras

> [!IMPORTANT] Aprendizaje Basado en la Práctica
> La mejor manera de aprender RxJS es **construyendo características reales**. Empieza con un patrón que coincida con tus necesidades actuales del proyecto, copia el ejemplo de código y luego adáptalo paso a paso.

## Secciones Relacionadas

- **[Capítulo 10: Anti-Patrones](../anti-patterns/index.md)** - Aprende qué evitar
- **[Capítulo 11: Superar Dificultades](../overcoming-difficulties/index.md)** - Abordar desafíos conceptuales
- **[Apéndice: Ecosistema](../appendix/rxjs-and-reactive-streams-ecosystem.md)** - Explorar bibliotecas e integraciones de frameworks

## Recursos de Referencia

- [Documentación Oficial de RxJS](https://rxjs.dev) - Referencia API y guías
- [Learn RxJS](https://www.learnrxjs.io/) - Ejemplos de operadores y patrones
- [RxJS Marbles](https://rxmarbles.com/) - Diagramas interactivos de operadores
- [RxJS Visualizer](https://rxviz.com/) - Visualiza flujos de datos RxJS
