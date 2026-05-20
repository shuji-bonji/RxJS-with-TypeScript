---
description: Esta sección cubre las Creation Functions para la creación básica de Observable, utilizando of, from, fromEvent, interval y timer para crear Observables desde una variedad de fuentes de datos que incluyen valores únicos, arrays, Promises, eventos y temporizadores. Este es un concepto importante que se puede implementar con seguridad de tipos TypeScript y es la base de RxJS.
---

# Creation Functions básicas

Las Creation Functions más básicas y utilizadas con frecuencia. Cree fácilmente Observables basados en datos, arrays, eventos y tiempo.

## ¿Qué son las Creation Functions básicas?

Las Creation Functions básicas son funciones para crear un único Observable desde una variedad de fuentes de datos. Son el conjunto de funciones más fundamental en RxJS y se utilizan en casi todo el código RxJS.

Revise la tabla a continuación para ver las características y el uso de cada Creation Function.

## Principales Creation Functions básicas

| Función | Descripción | Casos de uso |
|----------|------|-------------|
| **[of](/es/guide/creation-functions/basic/of)** | Emitir valores especificados en secuencia | Pruebas con valores fijos, creación de mocks |
| **[from](/es/guide/creation-functions/basic/from)** | Convertir desde array, Promise, etc. | Streaming de datos existentes |
| **[fromEvent](/es/guide/creation-functions/basic/fromEvent)** | Convertir eventos a Observable | Eventos DOM, Node.js EventEmitter |
| **[interval](/es/guide/creation-functions/basic/interval)** | Emitir continuamente a intervalos especificados | Polling, ejecución periódica |
| **[timer](/es/guide/creation-functions/basic/timer)** | Comenzar a emitir después de un retraso | Ejecución retardada, timeout |

## Criterios de uso

La elección de las Creation Functions básicas se determina por el tipo de fuente de datos.

### 1. Tipo de datos

- **Valores estáticos**: `of()` - Crea un Observable especificando el valor directamente
- **Array o Iterable**: `from()` - Convertir una colección existente en un stream
- **Promise**: `from()` - Convertir procesamiento asíncrono a Observable
- **Evento**: `fromEvent()` - Convierte un event listener a un Observable
- **Basado en tiempo**: `interval()`, `timer()` - Emitir valores basados en el paso del tiempo

### 2. Tiempo de emisión

- **Emitir inmediatamente**: `of()`, `from()` - Comenzar a emitir valores al suscribirse
- **Al ocurrir evento**: `fromEvent()` - Emitir cuando ocurre un evento
- **Emitir periódicamente**: `interval()` - Emitir continuamente a intervalos regulares
- **Emitir después de retraso**: `timer()` - Comenzar a emitir después de un tiempo especificado

### 3. Tiempo de finalización

- **Completar inmediatamente**: `of()`, `from()` - Completar después de que se emitan todos los valores
- **Nunca completar**: `fromEvent()`, `interval()` - Continuar hasta unsubscribe
- **Emitir una vez y completar**: `timer(delay)` - Completar después de emitir un valor

## Ejemplos de uso práctico

### of() - Pruebas con valores fijos

```typescript
import { of } from 'rxjs';

// Crear datos de prueba
const mockUser$ = of({ id: 1, name: 'Usuario de prueba' });

mockUser$.subscribe(user => console.log(user));
// Salida: { id: 1, name: 'Usuario de prueba' }
```

### from() - Streaming de un array

```typescript
import { from } from 'rxjs';
import { map } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

numbers$.pipe(
  map(n => n * 2)
).subscribe(console.log);
// Salida: 2, 4, 6, 8, 10
```

### fromEvent() - Evento de clic

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(() => console.log('¡Botón clickeado!'));
```

### interval() - Polling

```typescript
import { interval } from 'rxjs';
import { switchMap } from 'rxjs';

// Polling de API cada 5 segundos
interval(5000).pipe(
  switchMap(() => fetchData())
).subscribe(data => console.log('Actualizado:', data));
```

### timer() - Ejecución retardada

```typescript
import { timer } from 'rxjs';

// Ejecutar después de 3 segundos
timer(3000).subscribe(() => console.log('3 segundos pasados'));
```

## Cuidado con las fugas de memoria

La cancelación de suscripción adecuada es importante al usar las Creation Functions básicas.

> [!WARNING]
> `fromEvent()`, `interval()` y `timer(delay, period)` periódico no completarán y siempre deben cancelarse con `unsubscribe()` o automáticamente con `takeUntil()` o similar cuando se destruye el componente.
>
> Nota: `timer(delay)` sin segundo argumento completará automáticamente después de emitir una vez.

```typescript
import { fromEvent, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class MyComponent {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    fromEvent(window, 'resize').pipe(
      takeUntil(this.destroy$)
    ).subscribe(() => console.log('Ventana redimensionada'));
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

## Conversión de Cold a Hot

Como se muestra en la tabla anterior, **todas las Creation Functions básicas generan Cold Observables**. Cada suscripción inicia una ejecución independiente.

Sin embargo, puede **convertir un Cold Observable a un Hot Observable** utilizando los siguientes operadores multicast.

### Condiciones y operadores para conversión a Hot

| Operador | Comportamiento | Casos de uso |
|-------------|------|-------------|
| **share()** | Multicast + conexión/desconexión automática | Compartir solicitudes HTTP entre múltiples suscriptores |
| **shareReplay(n)** | Almacenar en caché los últimos n valores y entregar a nuevos suscriptores | Almacenar en caché respuestas de API |
| **publish() + connect()** | Iniciar multicast manualmente | Iniciar ejecución cuando los suscriptores estén listos |
| **multicast(subject)** | Multicast con Subject personalizado | Cuando se necesita control avanzado |

### Ejemplo práctico

```typescript
import { interval } from 'rxjs';
import { take, share } from 'rxjs';

// ❄️ Cold - Temporizador independiente para cada suscripción
const cold$ = interval(1000).pipe(take(3));

cold$.subscribe(val => console.log('A:', val));
setTimeout(() => {
  cold$.subscribe(val => console.log('B:', val));
}, 1500);

// Salida:
// A: 0 (después de 0s)
// A: 1 (después de 1s)
// B: 0 (después de 1.5s) ← B comienza independientemente desde 0
// A: 2 (después de 2s)
// B: 1 (después de 2.5s)

// 🔥 Hot - Comparte temporizador entre suscriptores
const hot$ = interval(1000).pipe(take(3), share());

hot$.subscribe(val => console.log('A:', val));
setTimeout(() => {
  hot$.subscribe(val => console.log('B:', val));
}, 1500);

// Salida:
// A: 0 (después de 0s)
// A: 1 (después de 1s)
// A: 2, B: 2 (después de 2s) ← B se une a mitad de camino, recibe el mismo valor
```

> [!TIP]
> **Casos donde se requiere conversión Hot**:
> - Quiere compartir una solicitud HTTP entre múltiples suscriptores
> - Quiere mantener solo una conexión WebSocket o de servidor
> - Quiere usar los resultados de cálculos de alto costo en múltiples ubicaciones
>
> Para más información, consulte el capítulo **Subject y Multicast** (Capítulo 5).

## Relación con Pipeable Operator

No existe un Pipeable Operator que corresponda directamente a las Creation Functions básicas. Siempre se utilizan como Creation Functions.

Sin embargo, se utilizan en combinación con Pipeable Operators en el siguiente patrón:

```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, switchMap } from 'rxjs';

// Entrada de usuario → Esperar 300ms → Llamada API
fromEvent(input, 'input').pipe(
  debounceTime(300),
  switchMap(event => fetchSuggestions(event.target.value))
).subscribe(suggestions => console.log(suggestions));
```

## Próximos pasos

Para obtener más información sobre cómo funciona cada Creation Function y ejemplos prácticos, haga clic en los enlaces de la tabla anterior.

Además, al aprender [Creation Functions de combinación](/es/guide/creation-functions/combination/), [Creation Functions de selection/partition](/es/guide/creation-functions/selection/) y [Creation Functions condicionales](/es/guide/creation-functions/conditional/), puede entender el panorama general de las Creation Functions.
