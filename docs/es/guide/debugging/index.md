---
description: Esta página proporciona explicaciones prácticas de las técnicas de depuración de RxJS desde las perspectivas de estrategias básicas, escenarios comunes de depuración, herramientas de depuración y depuración de rendimiento.
---

# Técnicas de depuración de RxJS

Debido a la naturaleza de los streams asincrónicos, la depuración de RxJS requiere un enfoque diferente al de las técnicas tradicionales de depuración sincrónica.

Esta página proporciona estrategias básicas para depurar aplicaciones RxJS y navegación hacia técnicas detalladas de depuración.

## Descripción general de las técnicas de depuración

La depuración de RxJS se puede categorizar en los siguientes cuatro enfoques

| Enfoque | Contenido | Página de detalle |
|----------|------|-----------|
| **Estrategia básica** | operador tap, herramientas de desarrollo, RxJS DevTools | Explicado en esta página |
| **Escenarios comunes** | Seis problemas típicos: no fluyen valores, fugas de memoria, errores no detectados, etc. | [→ Detalles](/es/guide/debugging/common-scenarios) |
| **Herramientas personalizadas** | Streams con nombre, operadores de depuración, medición de rendimiento | [→ Detalles](/es/guide/debugging/custom-tools) |
| **Rendimiento** | Monitoreo de suscripciones, detección de reevaluación, verificación de uso de memoria, mejores prácticas | [→ Detalles](/es/guide/debugging/performance) |

## Estrategias básicas de depuración

### 1. Salida de log con el operador `tap`

El operador `tap` es la técnica de depuración más básica, que permite observar los valores del stream sin efectos secundarios.

```ts
import { interval } from 'rxjs';
import { map, filter, tap } from 'rxjs';

interval(1000)
  .pipe(
    tap(value => console.log('🔵 Valor original:', value)),
    map(x => x * 2),
    tap(value => console.log('🟢 Después de map:', value)),
    filter(x => x > 5),
    tap(value => console.log('🟡 Después de filter:', value))
  )
  .subscribe(value => console.log('✅ Valor final:', value));

// Salida:
// 🔵 Valor original: 0
// 🟢 Después de map: 0
// 🔵 Valor original: 1
// 🟢 Después de map: 2
// 🔵 Valor original: 2
// 🟢 Después de map: 4
// 🔵 Valor original: 3
// 🟢 Después de map: 6
// 🟡 Después de filter: 6
// ✅ Valor final: 6
```

#### Puntos clave
- Inserte un `tap` en cada paso del pipeline para rastrear el flujo de datos
- Use pictogramas y etiquetas para mejorar la visibilidad del log
- Los logs de depuración se pueden insertar de forma segura porque `tap` no cambia los valores

### 2. Información de log detallada

Para obtener información de depuración más detallada, use el objeto Observer.

```ts
import { of, throwError, concat } from 'rxjs';
import { tap } from 'rxjs';

const debug = (tag: string) =>
  tap({
    next: value => console.log(`[${tag}] next:`, value),
    error: error => console.error(`[${tag}] error:`, error),
    complete: () => console.log(`[${tag}] complete`)
  });

// Stream normal
of(1, 2, 3)
  .pipe(debug('Normal'))
  .subscribe();

// Salida:
// [Normal] next: 1
// [Normal] next: 2
// [Normal] next: 3
// [Normal] complete

// Stream con error
concat(
  of(1, 2),
  throwError(() => new Error('Ocurrió un error'))
)
  .pipe(debug('Error'))
  .subscribe({
    error: () => {} // Manejo de errores
  });

// Salida:
// [Error] next: 1
// [Error] next: 2
// [Error] error: Error: Ocurrió un error
```

### 3. Verificación con herramientas de desarrollo

Esta es una técnica de depuración que utiliza las herramientas de desarrollo del navegador.

```ts
import { fromEvent, timer } from 'rxjs';
import { map, tap, debounceTime } from 'rxjs';

// Función auxiliar para depuración
function tapDebugger<T>(label: string) {
  return tap<T>({
    next: value => {
      console.group(`🔍 ${label}`);
      console.log('Valor:', value);
      console.log('Tipo:', typeof value);
      console.log('Timestamp:', new Date().toISOString());
      console.trace('Stack trace');
      console.groupEnd();
    }
  });
}

// Depuración de eventos de clic de botón
const button = document.querySelector('button');
if (button) {
  fromEvent(button, 'click')
    .pipe(
      tapDebugger('Click Event'),
      debounceTime(300),
      tapDebugger('After Debounce'),
      map(() => ({ timestamp: Date.now() }))
    )
    .subscribe(data => console.log('📤 Enviar:', data));
}
```

#### Utilización de herramientas de desarrollo
- Agrupar logs con `console.group()`
- Mostrar trazas de pila con `console.trace()`
- Mostrar arrays y objetos en un formato fácil de leer con `console.table()`
- Colocar puntos de interrupción en `tap`

### 4. Utilización de RxJS DevTools

RxJS DevTools es una herramienta de depuración proporcionada como extensión del navegador.

#### Instalación
- Chrome: [RxJS DevTools - Chrome Web Store](https://chrome.google.com/webstore)
- Firefox: [RxJS DevTools - Firefox Add-ons](https://addons.mozilla.org/)

#### Características principales
- Visualización del estado de suscripción de Observable
- Visualización de línea de tiempo de valores del stream
- Detección de fugas de memoria
- Análisis de rendimiento

#### Ejemplo de uso

```ts
import { interval } from 'rxjs';
import { take, map } from 'rxjs';

// Habilitar depuración solo en entorno de desarrollo
// Diferentes herramientas de compilación usan diferentes verificaciones de variables de entorno
const isDevelopment =
  // Vite: import.meta.env.DEV
  // webpack: process.env.NODE_ENV === 'development'
  // Configuración manual: usar variables globales
  typeof window !== 'undefined' && (window as any).__DEV__ === true;

const stream$ = interval(1000).pipe(
  take(5),
  map(x => x * 2)
);

if (isDevelopment) {
  // Hacer observable en DevTools
  stream$.subscribe({
    next: value => console.log('DevTools:', value)
  });
}
```

## Técnicas de depuración detalladas

Una vez que comprenda la estrategia básica, aprenda técnicas de depuración específicas en las páginas detalladas a continuación.

### Escenarios comunes de depuración

Seis problemas típicos encontrados en el desarrollo del mundo real y cómo resolverlos

- Escenario 1: No fluyen valores
- Escenario 2: Se emite un valor diferente al esperado
- Escenario 3: La suscripción nunca se completa (stream infinito)
- Escenario 4: Fuga de memoria (olvidó cancelar la suscripción)
- Escenario 5: Ocurre un error y no se detecta
- Escenario 6: Quiero rastrear los intentos de reintento

[→ Ver escenarios comunes de depuración](/es/guide/debugging/common-scenarios)

### Herramientas de depuración personalizadas

Cómo crear sus propias herramientas de depuración para satisfacer los requisitos de su proyecto

- Depuración de streams con nombre (tagStream)
- Creación de operadores de depuración personalizados
- Operador para medición de rendimiento (measure)

[→ Ver herramientas de depuración personalizadas](/es/guide/debugging/custom-tools)

### Depuración de rendimiento

Optimización de aplicaciones y mejores prácticas

- Verificar y rastrear suscripciones
- Detectar reevaluaciones innecesarias (shareReplay)
- Monitorear el uso de memoria
- Creación de un entorno de depuración
- Depuración con seguridad de tipos
- Establecer límites de error

[→ Ver depuración de rendimiento](/es/guide/debugging/performance)

## Resumen

La depuración de RxJS se puede realizar de manera eficiente siguiendo estos puntos.

### Estrategia básica
- ✅ Observe cada etapa del stream con el operador `tap`
- ✅ Utilice herramientas de desarrollo para obtener logs detallados
- ✅ Visualice el stream con RxJS DevTools

### Escenarios comunes
- ✅ Los valores no fluyen → Olvidó la suscripción, verifique las condiciones de filtrado
- ✅ Valor diferente al esperado → Orden de operadores, tenga en cuenta el compartir referencias
- ✅ Suscripción no completada → use `take` o `takeUntil` para streams infinitos
- ✅ Fugas de memoria → cancelación automática de suscripción con patrón `takeUntil`
- ✅ Errores no detectados → implemente un manejo de errores adecuado

### Herramientas de depuración
- ✅ Depuración flexible con operadores de depuración personalizados
- ✅ Rastree múltiples streams con streams con nombre
- ✅ Identifique cuellos de botella con medición de rendimiento

### Rendimiento
- ✅ Prevenga fugas de memoria monitoreando las suscripciones
- ✅ Evite recálculos innecesarios con `shareReplay`
- ✅ Verifique el uso de memoria periódicamente

Combinadas, estas técnicas permiten una depuración eficiente de las aplicaciones RxJS.

## Páginas relacionadas

- [Manejo de errores](/es/guide/error-handling/strategies) - Estrategias de manejo de errores
- [Técnicas de pruebas](/es/guide/testing/unit-tests) - Cómo probar RxJS
- [Anti-patrones de RxJS](/es/guide/anti-patterns/) - Errores comunes y soluciones
- [Pipeline](/es/guide/operators/pipeline) - Encadenamiento de operadores
