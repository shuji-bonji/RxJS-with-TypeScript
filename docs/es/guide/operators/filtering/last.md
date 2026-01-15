---
description: "El operador last recupera solo el último valor al completar el flujo o el último valor que coincide con una condición: Esencial para la extracción del estado final"
titleTemplate: ':title'
---

# last - Obtener Último Valor

El operador `last` recupera el **último valor** o **último valor que satisface una condición** del flujo y completa el flujo.


## 🔰 Sintaxis Básica y Uso

```ts
import { from } from 'rxjs';
import { last } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

// Obtener solo el último valor
numbers$.pipe(
  last()
).subscribe(console.log);

// Obtener solo el último valor que satisface la condición
numbers$.pipe(
  last(n => n < 5)
).subscribe(console.log);

// Salida:
// 5
// 4
```

- `last()` emite el **último valor emitido** al completar el flujo.
- Si se pasa una condición, solo se recuperará el **último valor** que satisface la condición.
- Si no existe ningún valor que coincida con la condición, se genera un error.

[🌐 Documentación Oficial de RxJS - `last`](https://rxjs.dev/api/operators/last)


## 💡 Patrones de Uso Típicos

- Obtener el último elemento de datos filtrados
- Recuperar el último estado al completar el flujo
- Recuperar la última operación significativa en el registro de sesión u operación


## 🧠 Ejemplo de Código Práctico (con UI)

Recuperar y mostrar el último valor que fue menor que 5 de los múltiples valores ingresados.

```ts
import { fromEvent } from 'rxjs';
import { map, filter, take, last } from 'rxjs';

// Crear área de salida
const output = document.createElement('div');
output.innerHTML = '<h3>Ejemplo Práctico de last:</h3>';
document.body.appendChild(output);

// Crear campo de entrada
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Ingrese un número y presione Enter';
document.body.appendChild(input);

// Flujo de evento de entrada
fromEvent<KeyboardEvent>(input, 'keydown')
  .pipe(
    filter((e) => e.key === 'Enter'),
    map(() => parseInt(input.value, 10)),
    take(5), // Completar cuando solo se tomen los primeros 5 valores
    filter((n) => !isNaN(n) && n < 5), // Solo pasar valores menores que 5
    last() // Obtener el último valor menor que 5
  )
  .subscribe({
    next: (value) => {
      const item = document.createElement('div');
      item.textContent = `Último valor menor que 5: ${value}`;
      output.appendChild(item);
    },
    complete: () => {
      const complete = document.createElement('div');
      complete.textContent = 'Completado';
      complete.style.fontWeight = 'bold';
      output.appendChild(complete);
    },
  });

```
1. Ingrese un número 5 veces y presione Enter
2. Seleccionar solo "menor que 5" de los números ingresados
3. Mostrar solo el último número ingresado que es menor que 5
4. El flujo se completa naturalmente y termina
