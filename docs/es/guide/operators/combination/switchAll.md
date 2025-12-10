---
description: switchAll es un operador que toma un Higher-order Observable (Observable de Observables), cambia a un nuevo Observable interno y cancela el antiguo.
---

# switchAll - Cambiar a Nuevo Observable Interno

El operador `switchAll` toma un **Higher-order Observable** (Observable de Observables),
**cambia cada vez que se emite un nuevo Observable interno**, y cancela el Observable interno antiguo.

## 🔰 Sintaxis Básica y Uso

```ts
import { fromEvent, interval } from 'rxjs';
import { map, switchAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Comenzar un nuevo contador para cada clic (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Cambiar a nuevo contador (cancelar contador antiguo)
higherOrder$
  .pipe(switchAll())
  .subscribe(x => console.log(x));

// Salida (con 3 clics):
// 0 (1er contador)
// 1 (1er contador)
// ← Clic aquí (1ro cancelado)
// 0 (2do contador) ← Cambiar a nuevo contador
// ← Clic aquí (2do cancelado)
// 0 (3er contador) ← Cambiar a nuevo contador
// 1 (3er contador)
// 2 (3er contador)
```

- Cuando se emite un nuevo Observable interno desde Higher-order Observable, **cambia inmediatamente**
- El Observable interno anterior se **cancela automáticamente**
- Solo el último Observable interno siempre está ejecutándose

[🌐 Documentación Oficial de RxJS - `switchAll`](https://rxjs.dev/api/index/function/switchAll)

## 💡 Patrones de Uso Típicos

- **Funcionalidad de búsqueda (cancelar búsquedas antiguas en cada entrada)**
- **Autocompletado**
- **Actualizaciones de datos en tiempo real (cambiar a última fuente de datos)**

## 🧠 Ejemplo de Código Práctico

Ejemplo de cancelar búsquedas antiguas y ejecutar solo la última búsqueda en cada entrada

```ts
import { fromEvent, of } from 'rxjs';
import { map, switchAll, debounceTime, delay } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Ingrese palabras clave de búsqueda';
document.body.appendChild(input);

const output = document.createElement('div');
document.body.appendChild(output);

let searchCount = 0;

// Debounce de eventos de entrada
const search$ = fromEvent(input, 'input').pipe(
  debounceTime(300),
  map((e) => (e.target as HTMLInputElement).value)
);

// Higher-order Observable: Llamada API de búsqueda simulada para cada valor de entrada
const results$ = search$.pipe(
  map(query => {
    const id = ++searchCount;
    const start = Date.now();

    // Llamada API de búsqueda simulada (retraso de 1 segundo)
    return of(`Resultados de búsqueda: "${query}"`).pipe(
      delay(1000),
      map(msg => {
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `[Búsqueda #${id}] ${msg} (${elapsed} segundos)`;
      })
    );
  }),
  switchAll() // Cancelar búsqueda antigua cuando comienza nueva
);

results$.subscribe(result => {
  output.innerHTML = ''; // Limpiar resultados anteriores
  const item = document.createElement('div');
  item.textContent = result;
  output.appendChild(item);
});
```

- **Las búsquedas antiguas se cancelan automáticamente** cuando el usuario cambia la entrada
- Solo se muestran siempre los últimos resultados de búsqueda

## 🔄 Operadores Relacionados

| Operador | Descripción |
|---|---|
| `switchMap` | Atajo para `map` + `switchAll` (más comúnmente usado) |
| [mergeAll](/es/guide/operators/combination/mergeAll) | Suscribirse a todos los Observables internos en paralelo |
| [concatAll](/es/guide/operators/combination/concatAll) | Suscribirse a Observables internos en orden (esperar completación anterior) |
| [exhaustAll](/es/guide/operators/combination/exhaustAll) | Ignorar nuevos Observables internos mientras se ejecuta |

## ⚠️ Notas Importantes

### Prevención de Fugas de Memoria

`switchAll` ayuda a prevenir fugas de memoria **cancelando automáticamente** Observables internos antiguos.
Es ideal para solicitudes frecuentes como búsquedas o autocompletado.

### Observables Internos que No Completan

Incluso si el Observable interno no completa, cambiará automáticamente cuando se emita un nuevo Observable interno.

```ts
// interval nunca completa, pero se cancela automáticamente en el siguiente clic
clicks$.pipe(
  map(() => interval(1000)), // Nunca completa
  switchAll()
).subscribe();
```

### Óptimo Cuando Solo Importa el Último Valor

Use `switchAll` cuando no necesite resultados de procesamiento antiguo y **solo el último resultado es importante**.
Si se necesitan todos los resultados, use [mergeAll](/es/guide/operators/combination/mergeAll).
