---
description: "El operador elementAt es un operador de filtrado de RxJS que sólo recupera valores en una posición de índice dada. Funciona de forma similar al acceso a índices de matrices."
---

# elementAt - Recuperado por especificación de índice

El operador `elementAt` recupera **sólo el valor en la posición del índice especificado** del Observable y completa el flujo inmediatamente. Funciona de forma similar a `array[index]` de un array.

## 🔰 Sintaxis básica y uso

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Salida.: 30(Índice2Valor)
```

**Flujo de operación**:.
1. 10 (índice 0) → saltar
2. 20 (índice 1) → saltar
3. 30 (índice 2) → salida y completo
4. 40, 50 no evaluados

[🌐 Documentación oficial de RxJS - `elementAt`](https://rxjs.dev/api/operators/elementAt)

## 💡 Patrón de utilización típico.

- **Paginación**: obtener el primer elemento de una página concreta.
- **Obtención de datos garantizados por orden**: obtener el enésimo evento o mensaje.
- **Pruebas y depuración**: validar el valor de una posición específica.
- **Acceso tipo array**: tratar Observable como un array.

## 🧠 Ejemplo práctico de código 1: Cuenta atrás de eventos

Este es un ejemplo de ejecución de una acción al enésimo clic.

```ts
import { fromEvent } from 'rxjs';
import { elementAt, map } from 'rxjs';

// UICrear
const output = document.createElement('div');
output.innerHTML = '<h3>5Haga clic una vez para mostrar el mensaje</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Haga clic en';
document.body.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'más5Hacer clic una vez';
output.appendChild(counter);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.fontSize = '18px';
result.style.fontWeight = 'bold';
output.appendChild(result);

let clickCount = 0;

// Evento de clic
const clicks$ = fromEvent(button, 'click');

// Para mostrar el recuento
clicks$.subscribe(() => {
  clickCount++;
  const remaining = 5 - clickCount;
  if (remaining > 0) {
    counter.textContent = `más${remaining}Hacer clic una vez`;
  } else {
    counter.textContent = '';
  }
});

// 5Segunda vez (índice)4Clics detectados de
clicks$.pipe(
  elementAt(4)
).subscribe(() => {
  result.textContent = '🎉 Alcanzado！';
  result.style.color = 'green';
  button.disabled = true;
});
```

- El quinto clic (índice 4) completa la acción.
- Empieza desde 0, igual que el índice del array.

## 🎯 Ejemplo práctico de código 2: Obtener el enésimo número del flujo de datos.

Este es un ejemplo de recuperación de un orden específico de valores a partir de datos publicados a intervalos regulares.

```ts
import { interval } from 'rxjs';
import { elementAt, map } from 'rxjs';

// UICrear
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Del flujo de datosNObtener el segundo';
container.appendChild(title);

const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Introduzca el índice (0〜del flujo de datos (9)';
input.min = '0';
input.max = '9';
input.style.marginRight = '10px';
container.appendChild(input);

const getButton = document.createElement('button');
getButton.textContent = 'Recuperar';
container.appendChild(getButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
container.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
result.style.display = 'none';
container.appendChild(result);

// del flujo de datos (0.5Los valores se emiten cada segundo,10hasta 1)
const data$ = interval(500).pipe(
  map(i => ({ index: i, value: Math.floor(Math.random() * 100), timestamp: Date.now() }))
);

getButton.addEventListener('click', () => {
  const index = parseInt(input.value);

  if (isNaN(index) || index < 0 || index > 9) {
    status.textContent = '0〜del flujo de datos (9Introduzca un rango de';
    status.style.color = 'red';
    return;
  }

  status.textContent = `Índice ${index} Se está recuperando el valor...`;
  status.style.color = 'blue';
  result.style.display = 'none';
  getButton.disabled = true;
  input.disabled = true;

  data$.pipe(
    elementAt(index)
  ).subscribe({
    next: data => {
      status.textContent = '';
      result.style.display = 'block';
      result.innerHTML = `
        <strong>✅ Recuperación correcta</strong><br>
        Índice: ${data.index}<br>
        Valor: ${data.value}<br>
        Marca de tiempo: ${new Date(data.timestamp).toLocaleTimeString()}
      `;
      result.style.color = 'green';
      result.style.backgroundColor = '#e8f5e9';
      getButton.disabled = false;
      input.disabled = false;
    },
    error: err => {
      status.textContent = '';
      result.style.display = 'block';
      result.textContent = `❌ Error: ${err.message}`;
      result.style.color = 'red';
      result.style.backgroundColor = '#ffebee';
      getButton.disabled = false;
      input.disabled = false;
    }
  });
});
```

- Recupera valores en un índice especificado de un flujo publicado cada 0,5 segundos.
- Se genera un error si el índice está fuera de rango.

## 🆚 Comparación con operadores similares

### elementAt vs take vs first

```ts
import { from } from 'rxjs';
import { elementAt, take, first, skip } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// elementAt: Sólo se recuperan los valores de un índice específico
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Salida.: 30

// take: Desde el principioNObtener un valor
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Salida.: 10, 20, 30

// skip + first: elementAt Equivalente a (redundante)
numbers$.pipe(
  skip(2),
  first()
).subscribe(console.log);
// Salida.: 30
```

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Salida.: 30(Índice2Valor)
---
description: elementAtオペレーターは、指定されたインデックス位置の値のみを取得するRxJSフィルタリングオペレーターです。配列のインデックスアクセスに似た動作をします。
---


## ⚠️ Notas.

### 1. si el índice está fuera de rango

Si no se alcanza el índice especificado antes de que se complete el flujo, se genera un error.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]); // 3Sólo uno

numbers$.pipe(
  elementAt(5) // Índice5Solicitar
).subscribe({
  next: console.log,
  error: err => console.error('Error:', err.message)
});
// Salida.: Error: no elements in sequence
```

### 2. Especificar valores por defecto.

Para evitar errores, se pueden especificar valores por defecto.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]);

// Especificar un valor por defecto
numbers$.pipe(
  elementAt(5, 999) // Índice5Si no está presente, devuelve999Devuelve un
).subscribe({
  next: console.log,
  error: err => console.error('Error:', err.message)
});
// Salida.: 999
```

### 3. Uso con flujos asíncronos

En flujos asíncronos, espere hasta que se alcance la posición del índice.

```ts
import { interval } from 'rxjs';
import { elementAt } from 'rxjs';

// 1Emite un valor cada segundo
interval(1000).pipe(
  elementAt(3) // Índice3(4(valor por segundo)
).subscribe(console.log);
// 3Emisión después de segundos: 3
```

### 4. No se permiten índices negativos

No se pueden especificar índices negativos.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ❌ Los índices negativos son errores
numbers$.pipe(
  elementAt(-1)
).subscribe({
  next: console.log,
  error: err => console.error('Error:', err.message)
});
// Error: ArgumentOutOfRangeError: index out of range
```

Utilice `takeLast` o `last` para obtener desde el final de la matriz.

```ts
import { from } from 'rxjs';
import { takeLast, last } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ✅ Obtener último valor
numbers$.pipe(
  last()
).subscribe(console.log);
// Salida.: 50

// ✅ Obtener el últimoNObtener el último valor
numbers$.pipe(
  takeLast(2)
).subscribe(console.log);
// Salida.: 40, 50
```

## 📚 Operadores relacionados.

- **[take](. /take)** - N tomado desde el principio.
- **[first](. /first)** - obtiene el primer valor.
- **[last](. /last)** - obtiene el último valor.
- **[skip](. /skip)** - salta los N primeros valores
- takeLast](. /takeLast)** - obtiene los N últimos valores

## Resumen.

El operador elementAt sólo recupera el valor en la posición de índice especificada.

- ✅ Mismo comportamiento que el acceso a índices de array.
- ✅ Ideal para recuperar el enésimo valor.
- ✅ Se pueden especificar valores por defecto para evitar errores
- ⚠️ Error si el índice está fuera de rango (sin valor por defecto)
- ⚠️ No se permiten índices negativos
- ⚠️ Los flujos asíncronos esperan hasta ser alcanzados
