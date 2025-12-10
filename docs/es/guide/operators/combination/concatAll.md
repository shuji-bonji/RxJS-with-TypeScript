---
description: concatAll es un operador que toma un Higher-order Observable (Observable de Observables) y aplana valores suscribiéndose a Observables internos en orden. Comienza el siguiente después de que el Observable anterior completa.
titleTemplate: ':title | RxJS'
---

# concatAll - Aplanar Observables Internos Secuencialmente

El operador `concatAll` toma un **Higher-order Observable** (Observable de Observables),
**se suscribe a Observables internos en orden**, y aplana sus valores. No comenzará el siguiente hasta que el Observable anterior complete.

## 🔰 Sintaxis Básica y Uso

```ts
import { fromEvent, interval } from 'rxjs';
import { map, concatAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Comenzar un nuevo contador para cada clic (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Suscribirse a contadores en orden (comenzar siguiente después de que el anterior completa)
higherOrder$
  .pipe(concatAll())
  .subscribe(x => console.log(x));

// Salida (con 3 clics):
// 0 (1er contador)
// 1 (1er contador)
// 2 (1er contador) ← Completo
// 0 (2do contador) ← Comenzar después de que el 1ro completa
// 1 (2do contador)
// 2 (2do contador) ← Completo
// 0 (3er contador) ← Comenzar después de que el 2do completa
// ...
```

- **Suscribirse en orden** a cada Observable interno emitido desde Higher-order Observable
- **No comenzar el siguiente** hasta que el Observable interno anterior complete
- El orden de los valores está garantizado

[🌐 Documentación Oficial de RxJS - `concatAll`](https://rxjs.dev/api/index/function/concatAll)

## 💡 Patrones de Uso Típicos

- **Ejecutar llamadas API en secuencia (ejecutar siguiente después de que la solicitud anterior completa)**
- **Reproducir animaciones en orden**
- **Procesar cargas de archivos secuencialmente**

## 🧠 Ejemplo de Código Práctico

Ejemplo de ejecución de llamadas API (simuladas) en orden para cada clic de botón

```ts
import { fromEvent, of } from 'rxjs';
import { map, concatAll, delay } from 'rxjs';

const button = document.createElement('button');
button.textContent = 'Llamada API';
document.body.appendChild(button);

const output = document.createElement('div');
document.body.appendChild(output);

let callCount = 0;

// Evento de clic de botón
const clicks$ = fromEvent(button, 'click');

// Higher-order Observable: Llamada API simulada para cada clic
const results$ = clicks$.pipe(
  map(() => {
    const id = ++callCount;
    const start = Date.now();

    // Llamada API simulada (retraso de 2 segundos)
    return of(`Llamada API #${id} completada`).pipe(
      delay(2000),
      map(msg => {
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `${msg} (${elapsed} segundos)`;
      })
    );
  }),
  concatAll() // Ejecutar todas las llamadas API en orden
);

results$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});
```

- Incluso con clics consecutivos de botón, **las llamadas API se ejecutan en orden**
- La siguiente llamada API comienza después de que la anterior completa

## 🔄 Operadores Relacionados

| Operador | Descripción |
|---|---|
| `concatMap` | Atajo para `map` + `concatAll` (comúnmente usado) |
| [mergeAll](/es/guide/operators/combination/mergeAll) | Suscribirse a todos los Observables internos en paralelo |
| [switchAll](/es/guide/operators/combination/switchAll) | Cambiar a nuevo Observable interno (cancelar antiguo) |
| [exhaustAll](/es/guide/operators/combination/exhaustAll) | Ignorar nuevos Observables internos mientras se ejecuta |

## ⚠️ Notas Importantes

### Backpressure (Acumulación de Cola)

Si la tasa de emisión del Observable interno es más rápida que la tasa de completación, **los Observables no procesados se acumularán en la cola**.

```ts
// Clic cada segundo → La llamada API toma 2 segundos
// → La cola puede acumularse continuamente
```

En este caso, considere estas contramedidas:
- Usar `switchAll` (procesar solo el último)
- Usar `exhaustAll` (ignorar durante la ejecución)
- Agregar debounce o throttling

### Cuidado con Observables Infinitos

Si el Observable anterior **nunca completa, el siguiente nunca comenzará**.

#### ❌ interval nunca completa, por lo que el 2do contador nunca comienza
```ts
clicks$.pipe(
  map(() => interval(1000)), // Nunca completa
  concatAll()
).subscribe();
```
#### ✅ Completar con take
```ts
clicks$.pipe(
  map(() => interval(1000).pipe(take(3))), // Completa después de 3
  concatAll()
).subscribe();
```
