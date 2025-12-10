---
description: mergeAll es un operador que toma un Higher-order Observable (Observable de Observables) y se suscribe a todos los Observables internos en paralelo para aplanar los valores.
titleTemplate: ':title | RxJS'
---

# mergeAll - Aplanar Todos los Observables Internos en Paralelo

El operador `mergeAll` toma un **Higher-order Observable** (Observable de Observables),
**se suscribe a todos los Observables internos en paralelo**, y aplana sus valores.

## 🔰 Sintaxis Básica y Uso

```ts
import { fromEvent, interval } from 'rxjs';
import { map, mergeAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Comenzar un nuevo contador para cada clic (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Suscribirse a todos los contadores en paralelo
higherOrder$
  .pipe(mergeAll())
  .subscribe(x => console.log(x));

// Salida (con 3 clics):
// 0 (1er contador)
// 1 (1er contador)
// 0 (2do contador) ← ejecución paralela
// 2 (1er contador)
// 1 (2do contador)
// 0 (3er contador) ← ejecución paralela
// ...
```

- **Suscribirse en paralelo** a cada Observable interno emitido desde Higher-order Observable
- **Combinar valores** de todos los Observables internos en un **solo stream**
- Se puede limitar el número de suscripciones concurrentes (`mergeAll(2)` = hasta 2 concurrentes)

[🌐 Documentación Oficial de RxJS - `mergeAll`](https://rxjs.dev/api/index/function/mergeAll)

## 💡 Patrones de Uso Típicos

- **Ejecutar múltiples llamadas API en paralelo**
- **Iniciar streams independientes para cada acción de usuario**
- **Integrar múltiples conexiones en tiempo real como WebSocket y EventSource**

## 🧠 Ejemplo de Código Práctico

Ejemplo de ejecución de llamadas API concurrentes (simuladas) en cada cambio de entrada

```ts
import { fromEvent, of } from 'rxjs';
import { map, mergeAll, delay, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Ingrese palabras clave de búsqueda';
document.body.appendChild(input);

const output = document.createElement('div');
document.body.appendChild(output);

// Debounce de eventos de entrada
const search$ = fromEvent(input, 'input').pipe(
  debounceTime(300),
  map((e) => (e.target as HTMLInputElement).value)
);

// Higher-order Observable: Llamada API simulada para cada valor de entrada
const results$ = search$.pipe(
  map(query =>
    // Llamada API simulada (retraso de 500ms)
    of(`Resultado: "${query}"`).pipe(delay(500))
  ),
  mergeAll() // Ejecutar todas las llamadas API en paralelo
);

results$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});
```

- **Todas las llamadas API se ejecutan en paralelo**, incluso si el usuario cambia rápidamente la entrada
- Los resultados de búsqueda antiguos pueden aparecer después de los nuevos resultados (sin garantía de orden)

## 🔄 Operadores Relacionados

| Operador | Descripción |
|---|---|
| `mergeMap` | Atajo para `map` + `mergeAll` (comúnmente usado) |
| [concatAll](/es/guide/operators/combination/concatAll) | Suscribirse a Observables internos en orden (esperar completación anterior) |
| [switchAll](/es/guide/operators/combination/switchAll) | Cambiar a nuevo Observable interno (cancelar antiguo) |
| [exhaustAll](/es/guide/operators/combination/exhaustAll) | Ignorar nuevos Observables internos mientras se ejecuta |

## ⚠️ Notas Importantes

### Limitar Suscripciones Concurrentes

No limitar las suscripciones concurrentes puede resultar en problemas de rendimiento.

```ts
// Limitar suscripciones concurrentes a 2
higherOrder$.pipe(
  mergeAll(2) // Hasta 2 ejecuciones concurrentes
).subscribe();
```

### Sin Garantía de Orden

Porque `mergeAll` ejecuta concurrentemente, **el orden de los valores no está garantizado**.
Si el orden es crítico, use [concatAll](/es/guide/operators/combination/concatAll).
