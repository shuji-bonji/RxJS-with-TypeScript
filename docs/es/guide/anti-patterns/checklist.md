---
description: "Lista de verificación para evitar anti-patrones en código RxJS: 16 mejores prácticas esenciales que cubren fugas de memoria, suscripciones, manejo de errores y selección de operadores"
---

# Lista de Verificación para Evitar Anti-patrones

Use esta lista de verificación para asegurarse de que su código RxJS no caiga en ningún anti-patrón. Haga clic en cada elemento para ver explicaciones detalladas y ejemplos de código.

## Elementos de la Lista de Verificación

### 🔴 Evitar Problemas Críticos

| Verificar | Elemento | Puntos Clave |
|:---:|---|---|
| <input type="checkbox" /> | **[Publicar Subject con asObservable()](./common-mistakes#1-publicacion-externa-de-subject)** | No exportar `Subject` directamente, publicarlo como Observable con `asObservable()`<br>Permitir cambios de estado solo a través de métodos dedicados |
| <input type="checkbox" /> | **[Evitar subscribe anidado](./common-mistakes#2-subscribe-anidado-callback-hell)** | No llamar otro `subscribe` dentro de `subscribe`<br>Aplanar con `switchMap`, `mergeMap`, `concatMap`, etc. |
| <input type="checkbox" /> | **[Siempre desuscribirse de streams infinitos](./common-mistakes#3-olvido-de-unsubscribe-fuga-de-memoria)** | Siempre desuscribirse de streams infinitos como event listeners<br>Patrón `takeUntil` o gestión de `Subscription` |
| <input type="checkbox" /> | **[Configurar explícitamente shareReplay](./common-mistakes#4-mal-uso-de-sharereplay)** | Usar la forma `shareReplay({ bufferSize: 1, refCount: true })`<br>Habilitar el conteo de referencias para prevenir fugas de memoria |
| <input type="checkbox" /> | **[Evitar sentencias if anidadas dentro de subscribe](./subscribe-if-hell)** | Evitar ramificación condicional compleja (3 o más niveles anidados) dentro de `subscribe`<br>Escribir declarativamente con operadores como `filter`, `iif`, `partition` |

### 🟡 Evitar Problemas que Requieren Atención

| Verificar | Elemento | Puntos Clave |
|:---:|---|---|
| <input type="checkbox" /> | **[map es función pura, efectos secundarios en tap](./common-mistakes#5-efectos-secundarios-en-map)** | No cambiar el estado ni mostrar logs dentro de `map`<br>Separar explícitamente los efectos secundarios con el operador `tap` |
| <input type="checkbox" /> | **[Usar Cold/Hot apropiadamente](./common-mistakes#6-ignorar-diferencias-entre-observable-cold-hot)** | Convertir solicitudes HTTP a Hot con `shareReplay`<br>Determinar si la ejecución debe ocurrir por suscripción o ser compartida |
| <input type="checkbox" /> | **[Convertir Promise con from](./promise-observable-mixing)** | No mezclar Promise y Observable<br>Convertir a Observable con `from()` para procesamiento unificado |
| <input type="checkbox" /> | **[Controlar eventos de alta frecuencia](./common-mistakes#8-ignorar-backpressure)** | Controlar entrada de búsqueda con `debounceTime`, desplazamiento con `throttleTime`<br>Excluir duplicados con `distinctUntilChanged` |

### 🔵 Mejorar la Calidad del Código

| Verificar | Elemento | Puntos Clave |
|:---:|---|---|
| <input type="checkbox" /> | **[Manejar errores apropiadamente](./common-mistakes#9-supresion-de-errores)** | Capturar errores con `catchError` y manejar apropiadamente<br>Mostrar mensajes de error amigables para el usuario<br>Reintentar con `retry` / `retryWhen` según sea necesario |
| <input type="checkbox" /> | **[Liberar eventos DOM correctamente](./common-mistakes#10-fugas-de-suscripcion-de-eventos-dom)** | Siempre desuscribirse de suscripciones `fromEvent`<br>Desuscribirse automáticamente con `takeUntil` cuando el componente se destruye |
| <input type="checkbox" /> | **[Asegurar la seguridad de tipos](./common-mistakes#11-falta-de-seguridad-de-tipos-uso-excesivo-de-any)** | Definir interfaces y alias de tipos<br>Especificar explícitamente parámetros de tipo `Observable<T>`<br>Aprovechar la inferencia de tipos de TypeScript |
| <input type="checkbox" /> | **[Elegir operadores apropiados](./common-mistakes#12-seleccion-impropia-de-operadores)** | Búsqueda: `switchMap`, paralelo: `mergeMap`<br>Secuencial: `concatMap`, prevenir doble clic: `exhaustMap` |
| <input type="checkbox" /> | **[El procesamiento simple no necesita RxJS](./common-mistakes#13-sobrecomplicacion)** | JavaScript regular es suficiente para procesamiento de arrays, etc.<br>Usar RxJS para procesamiento asíncrono y streams de eventos |
| <input type="checkbox" /> | **[Gestionar el estado reactivamente](./common-mistakes#14-cambios-de-estado-en-subscribe)** | Gestionar el estado con `BehaviorSubject` o `scan`<br>Usar `subscribe` como disparador final |
| <input type="checkbox" /> | **[Escribir pruebas](./common-mistakes#15-falta-de-pruebas)** | Implementar marble testing con `TestScheduler`<br>Hacer que el procesamiento asíncrono sea testeable de forma síncrona |

## Cómo Usar

### 1. Durante la Revisión de Código

Después de escribir código nuevo, realice una auto-revisión usando esta lista de verificación.

### 2. Durante Pull Requests

Incluya esta lista de verificación en su plantilla de pull request para que los revisores puedan verificar con criterios comunes.

### 3. Revisiones Regulares

Use esta lista de verificación regularmente contra su base de código existente para verificar anti-patrones.

### 4. Compartir Dentro del Equipo

Comparta con los miembros del equipo para unificar las mejores prácticas de RxJS.

## Recursos Relacionados

- **[Errores Comunes y Cómo Corregirlos](./common-mistakes)** - Explicaciones detalladas y ejemplos de código para cada anti-patrón
- **[Inicio de Colección de Anti-Patrones](./index)** - Lista de anti-patrones y cómo aprender
- **[Manejo de Errores](/es/guide/error-handling/strategies)** - Mejores prácticas de manejo de errores
- **[Técnicas de Prueba](/es/guide/testing/unit-tests)** - Cómo probar el código RxJS

## Consejos para Usar la Lista de Verificación

1. **No intente perfeccionar todos los elementos a la vez**
   - Primero, priorice los problemas críticos (🔴)
   - Mejore paso a paso

2. **Establezca prioridades dentro del equipo**
   - Ajuste la importancia según las características del proyecto
   - Cree listas de verificación personalizadas

3. **Considere la automatización**
   - Automatice verificaciones con herramientas de análisis estático como ESLint
   - Integre en el pipeline CI/CD

4. **Actualizaciones regulares**
   - Actualice según las actualizaciones de versión de RxJS
   - Refleje las ideas de la experiencia del equipo

---

**Importante**: Esta lista de verificación no es para escribir código perfecto, sino una guía para evitar problemas comunes. Úsela de forma flexible según el contexto de su proyecto.
