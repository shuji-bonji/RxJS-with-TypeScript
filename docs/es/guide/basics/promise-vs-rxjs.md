---
description: Comprende las diferencias entre Promise y RxJS y aprende a usarlos apropiadamente. Promise se especializa en procesamiento asíncrono único y se ejecuta inmediatamente, mientras que RxJS tiene evaluación perezosa, puede manejar múltiples valores, y puede cancelarse y reutilizarse. Esta guía explica en detalle las características de cada uno y los criterios para seleccionarlos a través de comparaciones de código y casos de uso específicos.
---

# Diferencias entre Promise y RxJS

## Resumen

Las principales herramientas para manejar procesamiento asíncrono en JavaScript/TypeScript son **Promise** y **RxJS (Observable)**. Aunque ambas se usan a veces para propósitos similares, su filosofía de diseño y casos de uso son bastante diferentes.

Esta página proporciona información para ayudarte a entender las diferencias entre Promise y RxJS y decidir cuál usar.

## Diferencias Básicas

| Elemento | Promise | RxJS (Observable) |
|------|---------|-------------------|
| **Estandarización** | Estándar JavaScript (ES6/ES2015) | Biblioteca de terceros |
| **Valores emitidos** | Valor único | Cero o más valores múltiples |
| **Evaluación** | Eager (se ejecuta inmediatamente al crearse) | Lazy (se ejecuta al suscribirse) |
| **Cancelación** | No posible[^1] | Posible (`unsubscribe()`) |
| **Reutilización** | No posible (el resultado es solo una vez) | Posible (puede suscribirse múltiples veces) |
| **Costo de aprendizaje** | Bajo | Alto (requiere entender operadores) |
| **Casos de uso** | Procesamiento asíncrono único | Procesamiento de streams complejo |

[^1]: Aunque el procesamiento basado en Promise (como fetch) puede cancelarse usando AbortController, la especificación de Promise en sí no tiene función de cancelación.

## Comparación de Código: Procesamiento Asíncrono Único

### Promise

```ts
// Promise se ejecuta inmediatamente al crearse (Eager)
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1')
  .then(response => response.json())
  .then(data => console.log(data))
  .catch(error => console.error(error));
```

Promise **comienza la ejecución en el momento en que se define** (evaluación Eager).

### RxJS

```ts
import { from } from 'rxjs';
import { switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

// Observable no se ejecuta hasta que se suscribe (Lazy)
const observable$ = from(fetch('https://jsonplaceholder.typicode.com/posts/1')).pipe(
  switchMap(response => response.json()), // response.json() devuelve una Promise, así que usa switchMap
  catchError(error => {
    console.error(error);
    return of(null);
  })
);

// La ejecución comienza solo cuando se suscribe
observable$.subscribe(data => console.log(data));
```

RxJS **no se ejecuta hasta que se llama `subscribe()`** (evaluación Lazy). Suscribirse al mismo Observable múltiples veces resulta en ejecuciones independientes, y el procesamiento puede interrumpirse con `unsubscribe()`.

> [!TIP]
> **Directrices de uso práctico**
> - Procesamiento inmediato de una sola vez → Promise
> - Procesamiento a ejecutar en un momento específico o múltiples veces → RxJS

## Comparación de Código: Manejo de Múltiples Valores

Una de las mayores diferencias entre Promise y RxJS es el número de valores que pueden emitirse. Promise solo puede devolver un valor único, mientras que RxJS puede emitir múltiples valores a lo largo del tiempo.

### Imposible con Promise

Promise solo puede **resolver una vez**.

```ts
// Promise solo puede devolver un valor único
const promise = new Promise(resolve => {
  resolve(1);
  resolve(2); // Este valor es ignorado
  resolve(3); // Este valor también es ignorado
});

promise.then(value => console.log(value));
// Salida: 1 (solo el primer valor)
```

Una vez que el valor se determina por el primer `resolve()`, las llamadas subsiguientes a `resolve()` son ignoradas.

### Posible con RxJS

Observable **puede emitir valores cualquier número de veces**.
```ts
import { Observable } from 'rxjs';

// Observable puede emitir múltiples valores
const observable$ = new Observable(subscriber => {
  subscriber.next(1);
  subscriber.next(2);
  subscriber.next(3);
  subscriber.complete();
});

observable$.subscribe(value => console.log(value));
// Salida: 1, 2, 3
```

Cada vez que se llama `next()`, el valor se entrega al suscriptor. Después de emitir todos los valores, se notifica la completación con `complete()`. Esta característica permite el manejo natural de datos que cambian en series temporales como comunicación en tiempo real, datos de streaming y procesamiento de eventos continuos.

> [!NOTE]
> **Ejemplos de aplicación práctica**
> - Recibir mensajes de WebSocket
> - Procesamiento secuencial de entrada de teclado
> - Streams de eventos del servidor (SSE)
> - Monitoreo continuo de datos de sensores

## Comparación de Cancelación

La capacidad de cancelar procesamiento asíncrono de larga duración o innecesario es importante desde las perspectivas de gestión de recursos y experiencia de usuario. Hay diferencias significativas en las capacidades de cancelación entre Promise y RxJS.

### Promise (No Cancelable)
Promise **no tiene función de cancelación estándar**.

```ts
const promise = new Promise(resolve => {
  setTimeout(() => resolve('Completo'), 3000);
});

promise.then(result => console.log(result));
// No hay forma estándar de cancelar este procesamiento
```

Una vez que comienza la ejecución, no puede detenerse hasta completarse, lo que puede causar fugas de memoria y degradación del rendimiento.

> [!WARNING]
> **Acerca de AbortController**
> APIs web como `fetch()` pueden cancelarse usando `AbortController`, pero esto no es una característica de Promise en sí, sino un mecanismo proporcionado por APIs individuales. No está disponible para todo el procesamiento asíncrono.

### RxJS (Cancelable)

RxJS **puede cancelarse en cualquier momento con `unsubscribe()`**.
```ts
import { timer } from 'rxjs';

const subscription = timer(3000).subscribe(
  () => console.log('Completo')
);

// Cancelar después de 1 segundo
setTimeout(() => {
  subscription.unsubscribe(); // Cancelar
  console.log('Cancelado');
}, 1000);
// Salida: Cancelado ("Completo" no se muestra)
```

Desuscribirse detiene inmediatamente el procesamiento en curso y previene fugas de memoria.

> [!TIP]
> **Casos de uso prácticos de cancelación**
> - Cancelar solicitudes HTTP cuando el usuario abandona la pantalla
> - Descartar resultados de consultas de búsqueda antiguas y procesar solo la última consulta (`switchMap`)
> - Cancelar automáticamente todos los Observables cuando el componente se destruye (patrón `takeUntil`)

## Cuál Elegir

Si usar Promise o RxJS depende de la naturaleza del procesamiento y los requisitos del proyecto. Usa los siguientes criterios como referencia para seleccionar la herramienta apropiada.

### Cuándo Elegir Promise

Promise es adecuado si se aplican las siguientes condiciones.

| Condición | Razón |
|------|------|
| Procesamiento asíncrono único | Una solicitud API, una lectura de archivo, etc. |
| Flujo de trabajo simple | `Promise.all`, `Promise.race` son suficientes |
| Proyectos de pequeña escala | Quiere minimizar dependencias |
| Usar solo API estándar | Quiere evitar bibliotecas externas |
| Código amigable para principiantes | Quiere reducir costos de aprendizaje |

#### Solicitud API Única:


```ts
interface User {
  id: number;
  name: string;
  email: string;
  username: string;
}

async function getUserData(userId: string): Promise<User> {
  const response = await fetch(`https://jsonplaceholder.typicode.com/users/${userId}`);
  if (!response.ok) {
    throw new Error('Error al recuperar datos del usuario');
  }
  return response.json();
}

// Ejemplo de uso
getUserData('1').then(user => {
  console.log('Nombre de usuario:', user.name);
  console.log('Email:', user.email);
});
```

Este código es un patrón típico para recuperar información de un usuario único. Usar `async/await` lo hace tan legible como código síncrono. El manejo de errores también puede unificarse con `try/catch`, haciéndolo simple e intuitivo.

#### Ejecución Paralela de Múltiples Procesos Asincrónicos:

```ts
interface Post {
  id: number;
  userId: number;
  title: string;
  body: string;
}

async function loadAllData(): Promise<[User[], Post[]]> {
  const [users, posts] = await Promise.all([
    fetch('https://jsonplaceholder.typicode.com/users').then(r => r.json()),
    fetch('https://jsonplaceholder.typicode.com/posts').then(r => r.json())
  ]);
  return [users, posts];
}

// Ejemplo de uso
loadAllData().then(([users, posts]) => {
  console.log('Número de usuarios:', users.length);
  console.log('Número de posts:', posts.length);
});
```

`Promise.all()` te permite ejecutar múltiples solicitudes API en paralelo y esperar a que todas se completen. Esto es muy conveniente para la carga inicial de datos. Ten en cuenta que si incluso una falla, todo el proceso da error, pero su simplicidad lo hace fácil de entender y mantener.

### Cuándo Elegir RxJS

RxJS es adecuado si se aplican las siguientes condiciones.

| Condición | Razón |
|------|------|
| Procesamiento continuo de eventos | Movimiento del ratón, entrada de teclado, WebSocket, etc. |
| Procesamiento de streams complejo | Combinar y transformar múltiples fuentes de eventos |
| Cancelación requerida | Quiere control fino de gestión de recursos |
| Reintento/Timeout | Quiere manejo de errores flexible |
| Proyectos Angular | RxJS está integrado en el framework |
| Datos en tiempo real | Los datos se actualizan continuamente |

#### Ejemplo Concreto
```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map, distinctUntilChanged, switchMap } from 'rxjs';

const label = document.createElement('label');
label.innerText = 'buscar: ';
const searchInput = document.createElement('input');
searchInput.type = 'input';
label.appendChild(searchInput);
document.body.appendChild(label);

// Búsqueda en tiempo real (autocompletado)
if (!searchInput) throw new Error('Input de búsqueda no encontrado');

fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300),              // Esperar 300ms antes de procesar
  distinctUntilChanged(),         // Procesar solo cuando el valor cambia
  switchMap(query =>              // Ejecutar solo la última solicitud
    fetch(`https://api.github.com/search/users?q=${query}`).then(r => r.json())
  )
).subscribe(results => {
  console.log('Resultados de búsqueda:', results.items); // La API de GitHub almacena resultados en la propiedad items
});
```

Este ejemplo es un caso típico donde RxJS muestra su verdadero valor. Monitorea la entrada del usuario, proporciona un tiempo de espera de 300ms para reducir solicitudes innecesarias, procesa solo cuando el valor cambia, y al hacer válida solo la última solicitud (`switchMap`), descarta automáticamente los resultados de solicitudes antiguas.

> [!IMPORTANT]
> **Por qué es difícil con Promise solo**
> - Debe implementar debounce manualmente (control de entrada continua)
> - Debe gestionar la cancelación de solicitudes antiguas usted mismo
> - Olvidar limpiar event listeners causa fugas de memoria
> - Debe rastrear múltiples estados simultáneamente (temporizadores, flags, gestión de solicitudes)
>
> Con RxJS, todo esto puede realizarse declarativamente en solo unas pocas líneas.

## Interoperabilidad entre Promise y RxJS

Promise y RxJS no son mutuamente excluyentes y pueden convertirse entre sí y combinarse. Esto es útil cuando se integra código existente basado en Promise en pipelines de RxJS, o a la inversa cuando quieres usar Observable en código existente basado en Promise.

## Convertir Promise a Observable

RxJS proporciona múltiples formas de convertir una Promise existente a Observable.

### Conversión por `from`

El método más común es usar `from`.

```ts
import { from } from 'rxjs';

// Crear Promise
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1')
  .then(response => response.json());

// Convertir a Observable con from()
const observable$ = from(promise);

observable$.subscribe({
  next: data => console.log('Datos:', data),
  error: error => console.error('Error:', error),
  complete: () => console.log('Completo')
});
```

El resultado de la Promise fluye como Observable, y la completación también se llama automáticamente.

### Conversión por `defer` (evaluación perezosa)

El `defer` retrasa la creación de una Promise hasta que se suscribe.

```ts
import { defer } from 'rxjs';

// Promise no se crea hasta subscribe
const observable$ = defer(() =>
  fetch('https://jsonplaceholder.typicode.com/posts/1').then(r => r.json())
);

// Crear nueva Promise en cada subscribe
observable$.subscribe(data => console.log('1ro:', data));
observable$.subscribe(data => console.log('2do:', data));
```

Este método es útil si quieres crear una nueva Promise cada vez que te suscribes.

## Convertir Observable a Promise

Es posible tomar solo un valor de un Observable y convertirlo en Promise.

### `firstValueFrom` y `lastValueFrom`

Las siguientes dos funciones se recomiendan en RxJS 7 y posteriores.

| Función | Comportamiento |
|------|------|
| `firstValueFrom` | Devuelve el primer valor como Promise |
| `lastValueFrom` | Devuelve el último valor al completarse como Promise |

```ts
import { of, firstValueFrom, lastValueFrom } from 'rxjs';
import { delay } from 'rxjs';

const observable$ = of(1, 2, 3).pipe(delay(1000));

// Obtener primer valor como Promise
const firstValue = await firstValueFrom(observable$);
console.log(firstValue); // 1

// Obtener último valor como Promise
const lastValue = await lastValueFrom(observable$);
console.log(lastValue); // 3
```

Si Observable se completa antes de que fluya el valor, por defecto es un error. Esto puede evitarse especificando un valor por defecto.

> [!WARNING]
> `toPromise()` está obsoleto. Usa `firstValueFrom()` o `lastValueFrom()` en su lugar.

> [!TIP]
> **Directrices de selección**
> - **`firstValueFrom()`**: Cuando solo se necesita el primer valor (ej., resultado de autenticación de login)
> - **`lastValueFrom()`**: Cuando se necesita el resultado final después de procesar todos los datos (ej., resultado de agregación)

## Ejemplo Práctico: Combinando Ambos

En el desarrollo real de aplicaciones, Promise y RxJS se combinan frecuentemente.

> [!WARNING] Precauciones Prácticas
> Mezclar Promise y Observable puede fácilmente **caer en anti-patrones si los límites de diseño no están claros**.
>
> **Problemas comunes:**
> - Se vuelve incancelable
> - Separación del manejo de errores
> - `await` dentro de `subscribe` (especialmente peligroso)
> - Adquisición paralela de los mismos datos con Promise y Observable
>
> Ver **[Capítulo 10: Anti-patrones de Mezcla de Promise y Observable](/es/guide/anti-patterns/promise-observable-mixing)** para detalles.

### Envío de Formulario y Llamadas API

Ejemplo de capturar el evento de envío de formulario de un usuario en RxJS y enviarlo al servidor usando Fetch API (Promise).

```ts
import { fromEvent, from } from 'rxjs';
import { exhaustMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface FormData {
  username: string;
  email: string;
}

// Envío de formulario basado en Promise
async function submitForm(data: FormData): Promise<{ success: boolean }> {
  const response = await fetch('https://api.example.com/submit', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  });
  if (!response.ok) {
    throw new Error('Envío fallido');
  }
  return response.json();
}

// Gestión de stream de eventos con RxJS
const submitButton = document.createElement('button');
submitButton.id = 'submit-button';
submitButton.innerText = 'Enviar';
submitButton.style.padding = '10px 20px';
submitButton.style.margin = '10px';
document.body.appendChild(submitButton);
if (!submitButton) throw new Error('Botón de envío no encontrado');

fromEvent(submitButton, 'click').pipe(
  exhaustMap(() => {
    const formData: FormData = {
      username: 'testuser',
      email: 'test@example.com'
    };
    // Convertir función Promise a Observable
    return from(submitForm(formData));
  }),
  catchError(error => {
    console.error('Error de envío:', error);
    return of({ success: false });
  })
).subscribe(result => {
  if (result.success) {
    console.log('Envío exitoso');
  } else {
    console.log('Envío fallido');
  }
});
```

Cada vez que se hace clic en el botón de envío del formulario, se inicia un nuevo proceso de envío, pero **ignora nuevos envíos durante el envío**.

En este ejemplo, el uso de `exhaustMap` previene solicitudes duplicadas durante la transmisión.

### Autocompletado de Búsqueda

Ejemplo de monitorear cambios en el valor del formulario de entrada y realizar búsquedas API.

```ts
import { fromEvent, from } from 'rxjs';
import { debounceTime, map, distinctUntilChanged, switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface SearchResult {
  items: Array<{
    login: string;
    id: number;
    avatar_url: string;
  }>;
  total_count: number;
}

// Función API basada en Promise
async function searchAPI(query: string): Promise<SearchResult> {
  const response = await fetch(`https://api.github.com/search/users?q=${query}`);
  if (!response.ok) {
    throw new Error('Búsqueda fallida');
  }
  return response.json();
}

// Gestión de stream de eventos con RxJS
const label = document.createElement('label');
label.innerText = 'buscar: ';
const searchInput = document.createElement('input');
searchInput.type = 'input';
label.appendChild(searchInput);
document.body.appendChild(label);
if (!searchInput) throw new Error('Input de búsqueda no encontrado');

fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300),
  distinctUntilChanged(),
  switchMap(query => {
    // Convertir función Promise a Observable
    return from(searchAPI(query));
  }),
  catchError(error => {
    console.error(error);
    return of({ items: [], total_count: 0 }); // Devolver resultado vacío en caso de error
  })
).subscribe(result => {
  console.log('Resultados de búsqueda:', result.items);
  console.log('Total:', result.total_count);
});
```

En este ejemplo, se realizan los siguientes controles:

- Esperar 300ms para completar la entrada con `debounceTime(300)`
- `distinctUntilChanged()` para ignorar si el valor es el mismo que el anterior
- `switchMap` para recuperar solo los últimos resultados de búsqueda (las solicitudes antiguas se cancelan automáticamente)

> [!WARNING] Cuidado con los anti-patrones
> El patrón de suscribir Observable en Promise puede causar fugas de memoria y comportamiento inesperado.
> <!-- TODO: Añadir enlace al anti-patrón subscribe-in-promise cuando esté disponible -->

> [!TIP]
> **Diseño por separación de responsabilidades**
>
> - **RxJS**: A cargo del control de eventos (debounce, switchMap, etc.)
> - **Promise**: A cargo de solicitudes HTTP (async/await)
> - **`from()`**: Puente entre ambos
>
> Usar cada tecnología apropiadamente mejora la legibilidad y mantenibilidad del código.

## Ventajas y Desventajas

Cada tecnología tiene su idoneidad y desventajas.

### Promise
<div class="comparison-cards">

::: tip Beneficios
- No requiere dependencias ya que es estándar JavaScript
- Código intuitivo y legible con `async/await`
- Bajo costo de aprendizaje
- Procesamiento simple de tareas únicas
:::

::: danger Desventajas
- No puede manejar múltiples valores
- Sin función de cancelación
- No adecuado para procesamiento de streams continuos
- El procesamiento de eventos complejo es difícil
:::

</div>

### RxJS
<div class="comparison-cards">

::: tip Beneficios
- Puede manejar múltiples valores a lo largo del tiempo
- Control complejo posible con una amplia variedad de operadores
- Cancelación (`unsubscribe`) es fácil
- Implementación flexible de manejo de errores y reintento
- Declarativo y testeable
:::

::: danger Desventajas
- Alto costo de aprendizaje
- Requiere bibliotecas
- Sobre-especificado para procesos simples
- La depuración puede ser difícil
:::

</div>

<style scoped>
.comparison-cards {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1rem;
  margin-bottom: 2rem;
}

@media (max-width: 768px) {
  .comparison-cards {
    grid-template-columns: 1fr;
  }
}
</style>

## Áreas Donde RxJS es Particularmente Activo

RxJS es particularmente poderoso en las siguientes áreas. Puede resolver elegantemente requisitos complejos que son difíciles de lograr solo con Promise.

| Área | Ejemplos | Comparación con Promise |
|------|----------|-------------------------|
| **Comunicación en Tiempo Real** | WebSocket, SSE, chat, actualizaciones de precios de acciones | Promise es solo para comunicación de una vez. No adecuado para procesamiento de mensajes continuos |
| **Control de Entrada de Usuario** | Autocompletado de búsqueda, validación de formularios | debounce, distinctUntilChanged, etc. son estándar |
| **Combinar Múltiples Fuentes** | Combinar condiciones de búsqueda × orden de clasificación × filtros | Puede describirse de manera concisa con combineLatest, withLatestFrom |
| **Soporte Offline** | PWA, monitoreo de estado de red, re-sincronización automática | Control de reintento flexible con retry, retryWhen |
| **APIs de Streaming** | OpenAI, salida secuencial de tokens de respuesta de IA | Puede procesar datos continuos en tiempo real |
| **Control de Cancelación** | Interrumpir procesos de larga duración, descartar solicitudes antiguas | Puede cancelar inmediatamente con unsubscribe() |

> [!NOTE]
> Para detalles sobre el uso de RxJS, ver también [¿Qué es RxJS - Casos de Uso](./what-is-rxjs.md#casos-de-uso).

## Resumen

| Propósito | Recomendado | Razón |
|------|------|------|
| Solicitud HTTP única | Promise (`async/await`) | Simple, legible, API estándar |
| Procesamiento de eventos de entrada de usuario | RxJS | Requiere control como debounce, distinct |
| Datos en tiempo real (WebSocket) | RxJS | Puede manejar naturalmente mensajes continuos |
| Ejecución paralela de múltiples procesos asincrónicos | Promise (`Promise.all`) | Promise es suficiente para ejecución paralela simple |
| Stream de eventos continuo | RxJS | Puede manejar múltiples valores a lo largo del tiempo |
| Procesamiento cancelable | RxJS | Cancelación confiable con unsubscribe() |
| Aplicaciones simples | Promise | Bajo costo de aprendizaje, pocas dependencias |
| Aplicaciones Angular | RxJS | Integrado estándar en el framework |

### Política Básica
- **Usa Promise si puede ser simple**
- **Usa RxJS si se requiere procesamiento de streams complejo**
- **Combinar ambos también es efectivo** (puente con `from()`)

RxJS es poderoso, pero no necesitas usar RxJS para todo el procesamiento asíncrono. Es importante usar la herramienta correcta en la situación correcta. Promise y RxJS son ambas herramientas poderosas para manejar procesamiento asíncrono, pero cada una tiene características diferentes.

- **Promise** es más adecuado para procesamiento asíncrono simple de una sola vez. Elige Promise para procesamiento asíncrono básico debido a su bajo costo de aprendizaje y buena compatibilidad con async/await.
- **RxJS** es poderoso cuando se requiere manejar múltiples valores, procesamiento de eventos o control de flujo de datos complejo. RxJS también es adecuado cuando se requieren controles avanzados como cancelar y reintentar.

En el desarrollo real, es importante usar ambos apropiadamente. Si es necesario, puedes ser flexible convirtiendo Promise a Observable u Observable a Promise.

> [!TIP] Próximos Pasos
> - Aprende más sobre Observable en [¿Qué es Observable](/es/guide/observables/what-is-observable)
> - Aprende cómo crear Observable en [Funciones de Creación](/es/guide/creation-functions/index)
> - Aprende cómo convertir y controlar Observables con [Operadores](/es/guide/operators/index)
