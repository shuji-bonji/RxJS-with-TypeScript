---
description: El operador isEmpty determina si Observable se ha completado sin emitir un valor y se usa para detección de datos vacíos y bifurcación condicional.
---

# isEmpty - Determinar si el Flujo está Vacío

El operador `isEmpty` **emite `true` si Observable se completa sin emitir ningún valor**.
Si emite incluso un valor, emite `false` y se completa.

## 🔰 Sintaxis Básica y Operación

```ts
import { of, EMPTY } from 'rxjs';
import { isEmpty } from 'rxjs';

EMPTY.pipe(isEmpty()).subscribe(console.log); // Salida: true
of(1).pipe(isEmpty()).subscribe(console.log); // Salida: false
```

[🌐 RxJS Official Documentation - isEmpty](https://rxjs.dev/api/index/function/isEmpty)

## 💡 Ejemplos de Uso Típicos

- Cuando quieres determinar si el resultado del filtrado o resultado de búsqueda está vacío
- Cuando quieres emitir un error o cambiar a otro proceso si está vacío

```ts
import { from } from 'rxjs';
import { filter, isEmpty } from 'rxjs';

from([1, 3, 5])
  .pipe(
    filter((x) => x % 2 === 0),
    isEmpty()
  )
  .subscribe((result) => {
    console.log('Está vacío:', result);
  });

// Salida:
// Está vacío: true
```

## 🧪 Ejemplos de Código Prácticos (con UI)

### ✅ 1. Determinar si el Resultado está Vacío

```ts
import { from } from 'rxjs';
import { filter, isEmpty } from 'rxjs';

const container = document.createElement('div');
container.innerHTML = '<h3>Ejemplo del operador isEmpty:</h3>';
document.body.appendChild(container);

const checkButton = document.createElement('button');
checkButton.textContent = 'Verificar si contiene números pares';
container.appendChild(checkButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.border = '1px solid #ccc';
container.appendChild(output);

checkButton.addEventListener('click', () => {
  from([1, 3, 5])
    .pipe(
      filter((x) => x % 2 === 0),
      isEmpty()
    )
    .subscribe((isEmptyResult) => {
      output.textContent = isEmptyResult
        ? 'No se encontraron números pares.'
        : 'Se incluyen números pares.';
      output.style.color = isEmptyResult ? 'red' : 'green';
    });
});
```

### ✅ 2. Verificar si los Resultados de Búsqueda de Usuario están Vacíos

```ts
import { fromEvent, of, from } from 'rxjs';
import { debounceTime, switchMap, map, filter, isEmpty, delay } from 'rxjs';

const searchContainer = document.createElement('div');
searchContainer.innerHTML = '<h3>Verificación de resultado de búsqueda con isEmpty:</h3>';
document.body.appendChild(searchContainer);

const input = document.createElement('input');
input.placeholder = 'Ingresa palabra de búsqueda';
input.style.marginBottom = '10px';
searchContainer.appendChild(input);

const resultBox = document.createElement('div');
resultBox.style.padding = '10px';
resultBox.style.border = '1px solid #ccc';
searchContainer.appendChild(resultBox);

const mockData = ['manzana', 'plátano', 'naranja', 'uva'];

fromEvent(input, 'input')
  .pipe(
    debounceTime(300),
    map((e) => (e.target as HTMLInputElement).value.trim().toLowerCase()),
    filter((text) => text.length > 0),
    switchMap((query) =>
      of(mockData).pipe(
        delay(300),
        map((list) => list.filter((item) => item.includes(query))),
        switchMap((filtered) => from(filtered).pipe(isEmpty()))
      )
    )
  )
  .subscribe((noResults) => {
    resultBox.textContent = noResults
      ? 'No se encontraron elementos coincidentes'
      : 'Se encontraron elementos coincidentes';
    resultBox.style.color = noResults ? 'red' : 'green';
  });
```

> [!WARNING] Atención en código de producción
> El ejemplo anterior omite la cancelación de suscripción de `fromEvent` para simplificar la explicación. En código real, gestione explícitamente el ciclo de vida con `takeUntil(destroy$)`, `take(N)`, o `Subscription.unsubscribe()`. Detalles: [Superar dificultades: gestión del ciclo de vida](/es/guide/overcoming-difficulties/lifecycle-management.md)
