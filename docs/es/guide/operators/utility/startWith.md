---
description: El operador startWith inserta el valor inicial especificado antes de que el Observable emita el valor, y es adecuado para la inicialización de estado y visualización inicial de la UI.
---

# startWith - Proporcionar Valor Inicial

El operador `startWith` es un operador para **emitir el valor inicial especificado antes de que el Observable fuente emita el valor**.
Se utiliza para gestión de estado, visualización inicial, valores de marcador de posición, etc.


## 🔰 Sintaxis Básica y Operación

```ts
import { of } from 'rxjs';
import { startWith } from 'rxjs';

of('B', 'C').pipe(
  startWith('A')
).subscribe(console.log);
// Salida:
// A
// B
// C
```

Así, `startWith` agrega `'A'` primero, seguido de los valores del Observable fuente.

[🌐 Documentación Oficial de RxJS - startWith](https://rxjs.dev/api/index/function/startWith)

## 💡 Ejemplo de Uso Típico

Esto es útil cuando desea establecer valores iniciales para estados o contadores. Aquí hay un ejemplo de un contador que comienza con un valor inicial de `100`.

```ts
import { interval } from 'rxjs';
import { startWith, scan, take } from 'rxjs';

interval(1000)
  .pipe(
    startWith(-1), // Insertar -1 primero
    scan((acc, curr) => acc + 1, 100), // Incrementar desde el valor inicial 100
    take(10) // Emitir 10 veces en total
  )
  .subscribe(console.log);
// Salida:
// 101
// 102
// 103
// 104
// 105
// 106
// 107
// 108
// 109
// 110
```


## 🧪 Ejemplo de Código Práctico (con UI)

```ts
import { interval } from 'rxjs';
import { startWith, scan, take } from 'rxjs';

// Área de visualización de salida
const startWithOutput = document.createElement('div');
startWithOutput.innerHTML = '<h3>Ejemplo de startWith:</h3>';
document.body.appendChild(startWithOutput);

// Área de visualización de contador
const counterDisplay = document.createElement('div');
counterDisplay.style.fontSize = '24px';
counterDisplay.style.fontWeight = 'bold';
counterDisplay.style.textAlign = 'center';
counterDisplay.style.padding = '20px';
counterDisplay.style.border = '1px solid #ddd';
counterDisplay.style.borderRadius = '5px';
counterDisplay.style.margin = '10px 0';
startWithOutput.appendChild(counterDisplay);

// Área de visualización de lista de valores
const valuesList = document.createElement('div');
valuesList.style.marginTop = '10px';
startWithOutput.appendChild(valuesList);

// Stream de contador (cada 1 segundo)
interval(1000)
  .pipe(
    // Comenzar con 100 primero
    startWith(-1),
    // Agregar 1 a cada valor al valor anterior
    scan((acc, curr) => acc + 1, 100),
    // Terminar después de 10 veces
    take(10)
  )
  .subscribe((count) => {
    // Actualizar visualización de contador
    counterDisplay.textContent = count.toString();

    // Agregar valor a la lista
    const valueItem = document.createElement('div');

    if (count === 100) {
      valueItem.textContent = `Valor inicial: ${count} (agregado con startWith)`;
      valueItem.style.color = 'blue';
    } else {
      valueItem.textContent = `Siguiente valor: ${count}`;
    }

    valuesList.appendChild(valueItem);
  });
```


## ✅ Resumen

- `startWith` es útil para situaciones donde desea **insertar un valor fijo primero**
- Comúnmente usado para inicialización de estado, marcadores de posición de UI, visualización inicial de formularios, etc.
- Usado en combinación con `scan`, `combineLatest`, etc. para **construir la base para gestión de estado**
