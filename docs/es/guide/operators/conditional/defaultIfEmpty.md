---
description: El operador defaultIfEmpty es un operador para devolver un valor por defecto si Observable no emite un valor, útil para manejo de datos vacíos y completado de valor inicial.
titleTemplate: ':title | RxJS'
---

# defaultIfEmpty - Valor por Defecto si el Flujo está Vacío

El operador `defaultIfEmpty` es un **operador que emite un valor por defecto especificado si Observable se completa sin emitir ningún valor**.
Se usa para lidiar con arrays vacíos o resultados de API vacíos.

## 🔰 Sintaxis Básica y Operación

```ts
import { from } from 'rxjs';
import { defaultIfEmpty } from 'rxjs';

from([]).pipe(
  defaultIfEmpty('Sin valor')
).subscribe(console.log);

// Salida:
// Sin valor
```

En este ejemplo, `defaultIfEmpty` emitirá `'Sin valor'` para un array vacío convertido en Observable con `from`.

[🌐 RxJS Official Documentation - defaultIfEmpty](https://rxjs.dev/api/index/function/defaultIfEmpty)

## 💡 Ejemplos de Uso Típicos

- Si el usuario no ingresó ninguna información
- Cuando la API devuelve un resultado vacío
- Si ninguno de los valores satisface las condiciones

Esto se usa para **completar la situación "no se devolvió nada"** en casos como estos.

```ts
import { of, EMPTY } from 'rxjs';
import { defaultIfEmpty, delay } from 'rxjs';

function mockApiCall(hasData: boolean) {
  return hasData
    ? of(['A', 'B', 'C']).pipe(delay(500))
    : EMPTY.pipe(delay(500));
}

mockApiCall(false)
  .pipe(defaultIfEmpty('Sin datos'))
  .subscribe(console.log);

// Salida:
// Sin datos
```

## 🧪 Ejemplos de Código Prácticos (con UI)

### ✅ 1. Usado para Determinar si un Array está Vacío

```ts
import { from } from 'rxjs';
import { defaultIfEmpty } from 'rxjs';

// Construir UI
const container = document.createElement('div');
container.innerHTML = '<h3>Ejemplo del operador defaultIfEmpty:</h3>';
document.body.appendChild(container);

const emptyBtn = document.createElement('button');
emptyBtn.textContent = 'Procesar array vacío';
container.appendChild(emptyBtn);

const nonEmptyBtn = document.createElement('button');
nonEmptyBtn.textContent = 'Procesar array no vacío';
container.appendChild(nonEmptyBtn);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
container.appendChild(result);

emptyBtn.addEventListener('click', () => {
  result.textContent = 'Procesando...';
  from([]).pipe(
    defaultIfEmpty('Sin datos')
  ).subscribe(value => {
    result.textContent = `Resultado: ${value}`;
  });
});

nonEmptyBtn.addEventListener('click', () => {
  result.textContent = 'Procesando...';
  from([1, 2, 3]).pipe(
    defaultIfEmpty('Sin datos')
  ).subscribe(value => {
    result.textContent = `Resultado: ${value}`;
  });
});
```

### ✅ 2. Completar Valor por Defecto para Resultado Vacío en API

```ts
import { of, EMPTY } from 'rxjs';
import { defaultIfEmpty, delay } from 'rxjs';

function mockApiCall(hasData: boolean) {
  return hasData
    ? of([
        { id: 1, name: 'Artículo 1' },
        { id: 2, name: 'Artículo 2' },
      ]).pipe(delay(1000))
    : EMPTY.pipe(delay(1000));
}

const apiContainer = document.createElement('div');
apiContainer.innerHTML = '<h3>Procesamiento de resultado de API con defaultIfEmpty:</h3>';
document.body.appendChild(apiContainer);

const dataBtn = document.createElement('button');
dataBtn.textContent = 'Con datos';
dataBtn.style.marginRight = '10px';
apiContainer.appendChild(dataBtn);

const emptyBtn2 = document.createElement('button');
emptyBtn2.textContent = 'Sin datos';
apiContainer.appendChild(emptyBtn2);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.border = '1px solid #ccc';
apiContainer.appendChild(output);

dataBtn.addEventListener('click', () => {
  output.textContent = 'Recuperando...';
  mockApiCall(true)
    .pipe(defaultIfEmpty('No se encontraron datos'))
    .subscribe({
      next: (val) => {
        if (Array.isArray(val)) {
          const ul = document.createElement('ul');
          val.forEach((item) => {
            const li = document.createElement('li');
            li.textContent = `${item.id}: ${item.name}`;
            ul.appendChild(li);
          });
          output.innerHTML = '<h4>Resultado:</h4>';
          output.appendChild(ul);
        } else {
          output.textContent = val;
        }
      },
    });
});

emptyBtn2.addEventListener('click', () => {
  output.textContent = 'Recuperando...';
  mockApiCall(false)
    .pipe(defaultIfEmpty('No se encontraron datos'))
    .subscribe({
      next: (val) => {
        output.textContent = val.toString();
      },
    });
});
```
