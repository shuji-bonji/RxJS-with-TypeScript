---
description: reduce es un operador de transformación de RxJS que acumula todos los valores del stream y emite solo el resultado final al completar. Es ideal para situaciones donde solo se necesita el resultado final de agregación, como cálculo de suma/promedio/máximo/mínimo de valores numéricos, agregación de objetos y construcción de arrays. A diferencia de scan, no emite resultados intermedios, y requiere que el stream se complete, por lo que no se puede usar con streams infinitos.
---

# reduce - Emitir solo el resultado acumulativo final

El operador `reduce` aplica una función acumulativa a cada valor del stream y **emite solo el resultado acumulativo final cuando el stream se completa**.
Funciona igual que `Array.prototype.reduce` de arrays, sin emitir resultados intermedios.

## 🔰 Sintaxis básica y uso

```ts
import { of } from 'rxjs';
import { reduce } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(reduce((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Salida: 15 (solo resultado final)
```

- `acc` es el valor acumulativo, `curr` es el valor actual.
- Comienza desde el valor inicial (en este caso `0`) y acumula secuencialmente.
- No emite valores hasta que el stream se completa, solo **emite el resultado final al completar**.

[🌐 Documentación oficial de RxJS - `reduce`](https://rxjs.dev/api/operators/reduce)

## 💡 Patrones de uso típicos

- Cálculo de suma, promedio, máximo, mínimo de valores numéricos
- Agregación y transformación de objetos
- Construcción y combinación de arrays
- Cuando solo se necesita el resultado final de agregación

## 🔍 Diferencia con scan

| Operador | Momento de emisión | Contenido emitido | Uso |
|:---|:---|:---|:---|
| `reduce` | **Solo una vez al completar** | Resultado acumulativo final | Agregación donde solo se necesita el resultado final |
| `scan` | **Cada vez con cada valor** | Todos incluidos resultados intermedios | Agregación en tiempo real, gestión de estado |

```ts
import { of } from 'rxjs';
import { reduce, scan } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== reduce ===');
source$.pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Salida: 15

console.log('=== scan ===');
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Salida: 1, 3, 6, 10, 15
```

## 🧠 Ejemplo de código práctico (con UI)

Ejemplo que suma los valores de múltiples campos de entrada y muestra el resultado final al hacer clic en un botón.

```ts
import { fromEvent, from } from 'rxjs';
import { map, reduce, switchMap } from 'rxjs';

// Crear campos de entrada
const inputs: HTMLInputElement[] = [];
for (let i = 1; i <= 3; i++) {
  const label = document.createElement('label');
  label.textContent = `Valor ${i}: `;
  const input = document.createElement('input');
  input.type = 'number';
  input.value = '0';
  label.appendChild(input);
  document.body.appendChild(label);
  document.body.appendChild(document.createElement('br'));
  inputs.push(input);
}

// Botón de cálculo
const button = document.createElement('button');
button.textContent = 'Calcular suma';
document.body.appendChild(button);

// Área de visualización de resultados
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Calcular suma al hacer clic en el botón
fromEvent(button, 'click').pipe(
  switchMap(() => {
    // Obtener todos los valores de entrada
    const values = inputs.map(input => Number(input.value) || 0);
    return from(values).pipe(
      reduce((acc, curr) => acc + curr, 0)
    );
  })
).subscribe(total => {
  output.textContent = `Suma: ${total}`;
  console.log('Suma:', total);
});
```

- Al hacer clic en el botón, se agregan todos los valores de entrada y solo se muestra la suma final.
- No se emiten resultados intermedios.

## 🎯 Ejemplo de agregación de objetos

Ejemplo práctico de agrupar múltiples valores en un objeto.

```ts
import { from } from 'rxjs';
import { reduce } from 'rxjs';

interface Product {
  category: string;
  price: number;
}

const products: Product[] = [
  { category: 'alimentos', price: 500 },
  { category: 'bebidas', price: 200 },
  { category: 'alimentos', price: 800 },
  { category: 'bebidas', price: 150 },
  { category: 'alimentos', price: 300 },
];

// Agregar total por categoría
from(products).pipe(
  reduce((acc, product) => {
    acc[product.category] = (acc[product.category] || 0) + product.price;
    return acc;
  }, {} as Record<string, number>)
).subscribe(result => {
  console.log('Total por categoría:', result);
});

// Salida:
// Total por categoría: { alimentos: 1600, bebidas: 350 }
```

## 🎯 Ejemplo de construcción de array

Ejemplo de agrupar valores del stream en un array.

```ts
import { interval } from 'rxjs';
import { take, reduce } from 'rxjs';

interval(100).pipe(
  take(5),
  reduce((acc, value) => {
    acc.push(value);
    return acc;
  }, [] as number[])
).subscribe(array => {
  console.log('Array recopilado:', array);
});

// Salida:
// Array recopilado: [0, 1, 2, 3, 4]
```

::: tip
Al construir arrays, considera usar el operador [`toArray`](../utility/toArray) más conciso.
```ts
interval(100).pipe(
  take(5),
  toArray()
).subscribe(console.log);
// Salida: [0, 1, 2, 3, 4]
```
:::

## 💡 Uso de reduce con seguridad de tipos

Ejemplo aprovechando la inferencia de tipos de TypeScript.

```ts
import { from } from 'rxjs';
import { reduce } from 'rxjs';

interface UserAction {
  type: 'click' | 'scroll' | 'input';
  timestamp: number;
}

const actions: UserAction[] = [
  { type: 'click', timestamp: 100 },
  { type: 'scroll', timestamp: 200 },
  { type: 'click', timestamp: 300 },
  { type: 'input', timestamp: 400 },
];

const actions$ = from(actions);

// Agregar cantidad por tipo de acción
actions$.pipe(
  reduce((acc, action) => {
    acc[action.type] = (acc[action.type] || 0) + 1;
    return acc;
  }, {} as Record<UserAction['type'], number>)
).subscribe(result => {
  console.log('Agregación de acciones:', result);
});

// Salida:
// Agregación de acciones: { click: 2, scroll: 1, input: 1 }
```

## ⚠️ Puntos de atención

### ❌ No se completa con streams infinitos (importante)

> [!WARNING]
> **`reduce` no emite ningún valor hasta que se llama a `complete()`.** Con streams infinitos (`interval`, `fromEvent`, etc.), nunca se obtiene un valor, lo que puede causar problemas en el trabajo real.

```ts
import { interval } from 'rxjs';
import { reduce } from 'rxjs';

// ❌ Mal ejemplo: no se emite valor con stream infinito
interval(1000).pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Sin salida (porque el stream no se completa)
```

**Solución 1: Usar `scan` si se necesita agregación continua**

```ts
import { interval, scan, take } from 'rxjs';

// ✅ Buen ejemplo: obtener resultados intermedios en tiempo real
interval(1000).pipe(
  take(5),
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Salida: 0, 1, 3, 6, 10 (emite valor acumulativo cada vez)
```

**Solución 2: `scan` + `takeLast(1)` si solo se necesita el valor final**

```ts
import { interval, scan, take, takeLast } from 'rxjs';

// ✅ Buen ejemplo: acumular con scan y obtener solo el valor final
interval(1000).pipe(
  take(5),
  scan((acc, curr) => acc + curr, 0),
  takeLast(1)
).subscribe(console.log);
// Salida: 10 (solo resultado final)
```

**Solución 3: Especificar condición de finalización con `take`**

```ts
import { interval, take, reduce } from 'rxjs';

// ✅ Buen ejemplo: establecer condición de finalización con take
interval(1000).pipe(
  take(5),
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Salida: 10
```

> [!TIP]
> **Criterios de selección**:
> - Se necesitan resultados intermedios → `scan`
> - Solo se necesita resultado final & se garantiza completación del stream → `reduce`
> - Solo se necesita resultado final & stream infinito → `scan` + `takeLast(1)` o `take` + `reduce`

### Uso de memoria

Se requiere precaución con el uso de memoria cuando el valor acumulativo se convierte en un objeto o array grande.

```ts
// Ejemplo que requiere atención al uso de memoria
from(largeDataArray).pipe(
  reduce((acc, item) => {
    acc.push(item); // Acumular grandes cantidades de datos
    return acc;
  }, [])
).subscribe();
```

## 📚 Operadores relacionados

- [`scan`](./scan) - Emitir resultados intermedios con cada valor
- [`toArray`](../utility/toArray) - Agrupar todos los valores en un array
- [`count`](https://rxjs.dev/api/operators/count) - Contar cantidad de valores
- [`min`](https://rxjs.dev/api/operators/min) / [`max`](https://rxjs.dev/api/operators/max) - Obtener valor mínimo/máximo

## Resumen

El operador `reduce` acumula todos los valores del stream y **emite solo el resultado final al completar**. Es apropiado cuando no se necesitan resultados intermedios y solo se necesita el resultado final de agregación. Sin embargo, como no se obtiene resultado si el stream no se completa, es necesario usar `scan` con streams infinitos o establecer condiciones de finalización con `take` u otros operadores.
