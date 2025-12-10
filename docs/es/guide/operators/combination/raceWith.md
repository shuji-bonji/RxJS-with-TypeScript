---
description: raceWith es un operador de combinación de RxJS que adopta solo el primer stream que emite un valor entre el Observable original y otros Observables. Es la versión Pipeable Operator de la Creation Function race, y es ideal para situaciones donde la respuesta más rápida es una prioridad, como implementaciones de timeout, adquisición paralela desde múltiples CDNs (fallback) y adquisición competitiva desde múltiples fuentes de datos. Es efectivo cuando se desea competir con otros streams mientras se convierte y procesa el stream principal.
titleTemplate: ':title | RxJS'
---

# raceWith - Adoptar el Stream Más Rápido (Dentro del Pipeline)

El operador `raceWith` **adopta solo el primer stream que emite un valor** entre el Observable original y los otros Observables especificados, e ignora todos los demás.
Esta es la versión Pipeable Operator de la Creation Function `race`.

## 🔰 Sintaxis Básica y Uso

```ts
import { interval, timer } from 'rxjs';
import { raceWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  take(3),
  map(val => `Fuente 1: ${val}`)
);

const source2$ = timer(500).pipe(
  take(3),
  map(val => `Fuente 2: ${val}`)
);

source1$
  .pipe(raceWith(source2$))
  .subscribe(console.log);

// Salida:
// Fuente 2: 0 (después de 500ms)
// * source1$ es ignorado porque source2$ emitió primero
```

- **El primer Observable en emitir un valor** gana la carrera, y solo ese stream es adoptado.
- Otros Observables se desuscriben automáticamente y se ignoran.

[🌐 Documentación Oficial de RxJS - `raceWith`](https://rxjs.dev/api/operators/raceWith)


## 💡 Patrones de Uso Típicos

- **Implementación de timeout**: Competir procesamiento principal con error de timeout después de cierto tiempo
- **Adquisición paralela desde múltiples CDNs**: Solicitar múltiples CDNs simultáneamente y adoptar la respuesta más rápida (estrategia de fallback)
- **Adquisición competitiva desde múltiples fuentes de datos**: Ejecutar caché local y llamada API concurrentemente, y usar el que regrese primero
- **Acción de usuario vs competición de temporizador**: Competir acción de clic con avance automático, y adoptar el que ocurra primero


## 🧠 Ejemplo de Código Práctico (con UI)

Ejemplo de obtención de datos desde múltiples CDNs en paralelo y adopción de la respuesta más rápida.

```ts
import { fromFetch } from 'rxjs/fetch';
import { raceWith, map, catchError, timeout } from 'rxjs';
import { of } from 'rxjs';

// Construir la UI
const container = document.createElement('div');
container.innerHTML = `
  <h3>Ejemplo Práctico de raceWith: Obtención Paralela desde Múltiples CDNs</h3>
  <button id="fetch-button">Obtener Datos</button>
  <div id="status" style="margin-top: 10px; padding: 10px; border: 1px solid #ccc;">
    Esperando...
  </div>
  <div id="result" style="margin-top: 10px;"></div>
`;
document.body.appendChild(container);

const fetchButton = document.getElementById('fetch-button') as HTMLButtonElement;
const statusDiv = document.getElementById('status')!;
const resultDiv = document.getElementById('result')!;

// Comenzar a obtener datos al hacer clic en el botón
fetchButton.addEventListener('click', () => {
  statusDiv.textContent = 'Obteniendo desde múltiples CDNs en paralelo...';
  statusDiv.style.backgroundColor = '#fff3e0';
  resultDiv.innerHTML = '';

  // Múltiples CDNs (en realidad endpoints ficticios)
  const cdn1$ = fromFetch('https://jsonplaceholder.typicode.com/posts/1').pipe(
    map(response => response.json()),
    map(() => ({ source: 'CDN 1', data: 'Datos obtenidos exitosamente' })),
    timeout(3000),
    catchError(() => of({ source: 'CDN 1', data: 'Error' }))
  );

  const cdn2$ = fromFetch('https://jsonplaceholder.typicode.com/posts/2').pipe(
    map(response => response.json()),
    map(() => ({ source: 'CDN 2', data: 'Datos obtenidos exitosamente' })),
    timeout(3000),
    catchError(() => of({ source: 'CDN 2', data: 'Error' }))
  );

  const cdn3$ = fromFetch('https://jsonplaceholder.typicode.com/posts/3').pipe(
    map(response => response.json()),
    map(() => ({ source: 'CDN 3', data: 'Datos obtenidos exitosamente' })),
    timeout(3000),
    catchError(() => of({ source: 'CDN 3', data: 'Error' }))
  );

  // ✅ Adoptar respuesta más rápida con raceWith
  cdn1$
    .pipe(raceWith(cdn2$, cdn3$))
    .subscribe({
      next: (result) => {
        statusDiv.textContent = `✅ Obtenido exitosamente desde ${result.source}`;
        statusDiv.style.backgroundColor = '#e8f5e9';
        resultDiv.innerHTML = `<strong>${result.source}</strong>: ${result.data}`;
      },
      error: (err) => {
        statusDiv.textContent = '❌ Falló al obtener desde todos los CDNs';
        statusDiv.style.backgroundColor = '#ffebee';
        resultDiv.textContent = `Error: ${err.message}`;
      }
    });
});
```

- Solicita múltiples CDNs simultáneamente, y **adopta el primer CDN** que devuelve una respuesta.
- Las respuestas de otros CDNs se ignoran automáticamente.


## 🔄 Diferencia con la Creation Function `race`

### Diferencias Básicas

| | `race` (Creation Function) | `raceWith` (Pipeable Operator) |
|:---|:---|:---|
| **Ubicación de Uso** | Usado como función independiente | Usado dentro de cadena `.pipe()` |
| **Sintaxis** | `race(obs1$, obs2$, obs3$)` | `obs1$.pipe(raceWith(obs2$, obs3$))` |
| **Primer Stream** | Trata todos por igual | Trata como stream principal |
| **Ventaja** | Simple y legible | Fácil de combinar con otros operadores |

### Ejemplos de Uso Específicos

**Creation Function Recomendada Solo para Competición Simple**

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const fast$ = timer(100).pipe(map(() => '¡Rápido gana!'));
const slow$ = timer(500).pipe(map(() => '¡Lento gana!'));

// Simple y legible
race(fast$, slow$).subscribe(console.log);
// Salida: ¡Rápido gana!
```

**Pipeable Operator Recomendado Cuando Se Agrega Procesamiento de Transformación al Stream Principal**

```ts
import { fromEvent, timer } from 'rxjs';
import { raceWith, map, mapTo, take } from 'rxjs';

// Clic de usuario vs competición de avance automático
const userClick$ = fromEvent(document, 'click').pipe(
  take(1),
  mapTo('Usuario hizo clic')
);

const autoAdvance$ = timer(5000).pipe(
  mapTo('Avanzó automáticamente')
);

// ✅ Versión Pipeable Operator - agregar procesamiento al stream principal
userClick$
  .pipe(
    map(message => `[${new Date().toLocaleTimeString()}] ${message}`),
    raceWith(autoAdvance$.pipe(
      map(message => `[${new Date().toLocaleTimeString()}] ${message}`)
    ))
  )
  .subscribe(console.log);

// ❌ Versión Creation Function - se vuelve verbosa
import { race } from 'rxjs';
race(
  userClick$.pipe(
    map(message => `[${new Date().toLocaleTimeString()}] ${message}`)
  ),
  autoAdvance$.pipe(
    map(message => `[${new Date().toLocaleTimeString()}] ${message}`)
  )
).subscribe(console.log);
```

### Resumen

- **`race`**: Óptimo para simplemente competir múltiples streams
- **`raceWith`**: Óptimo cuando se desea competir con otros streams mientras se transforma o procesa el stream principal


## ⚠️ Notas Importantes

### Primera Emisión Gana

El stream con el **tiempo de emisión más temprano** es adoptado. No el tiempo de inicio de suscripción.

```ts
import { timer, of } from 'rxjs';
import { raceWith, map } from 'rxjs';

const immediate$ = of('Emitir inmediatamente');
const delayed$ = timer(1000).pipe(map(() => 'Emitir después de 1 segundo'));

immediate$
  .pipe(raceWith(delayed$))
  .subscribe(console.log);
// Salida: Emitir inmediatamente
```

### Todos los Observables Se Suscriben

`raceWith` **se suscribe a todos los Observables simultáneamente**, pero ignora todos excepto el primero que emite.

```ts
import { timer } from 'rxjs';
import { raceWith, tap } from 'rxjs';

const source1$ = timer(100).pipe(
  tap(() => console.log('Fuente 1 emite'))
);

const source2$ = timer(200).pipe(
  tap(() => console.log('Fuente 2 emite'))
);

source1$
  .pipe(raceWith(source2$))
  .subscribe(console.log);
// Salida:
// Fuente 1 emite
// 0
// Fuente 2 emite ← Suscrito, pero el valor se ignora
```

### Manejo de Errores

Si hay un Observable que tiene error primero, todo el stream termina con un error.

```ts
import { throwError, timer } from 'rxjs';
import { raceWith, catchError } from 'rxjs';
import { of } from 'rxjs';

timer(1000).pipe(
  raceWith(
    throwError(() => new Error('Ocurrió un error')).pipe(
      catchError(err => of('Error recuperado'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error('Error:', err.message)
});
// Salida: Error recuperado
```


## 📚 Operadores Relacionados

- **[race](/es/guide/creation-functions/selection/race)** - Versión Creation Function
- **[mergeWith](/es/guide/operators/combination/mergeWith)** - Ejecutar todos los streams en paralelo
- **[concatWith](/es/guide/operators/combination/concatWith)** - Ejecutar streams secuencialmente
