---
description: "El operador findIndex es un operador de filtrado de RxJS que devuelve el índice del primer valor que satisface la condición. Si no se encuentra, devuelve -1."
---

# findIndex - obtiene el índice que coincide con la condición

El operador `findIndex` devuelve **el índice del primer valor que coincide con la condición** y completa el flujo inmediatamente. Devuelve `-1` si no se encuentra ningún valor.

## 🔰 Sintaxis básica y uso

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Salida.: 4(primer par8índice del primer par)
```

**Flujo de operaciones**:.
1. 1 (índice 0) → impar, saltar
2. 3 (índice 1) → impar, saltar
3. 5 (índice 2) → Impar, saltar
4. 7 (índice 3) → Impar, saltar
5. 8 (índice 4) → número par, salida índice 4 y completo

[🌐 Documentación oficial de RxJS - `findIndex`](https://rxjs.dev/api/operators/findIndex)

## 💡 Patrón de utilización típico.

- **Posicionamiento en un array**: obtención de la posición de un elemento que satisface una condición específica.
- **Comprobar el orden**: cuántas veces aparece un elemento que satisface una determinada condición.
- Reordenación de datos**: tratamiento a partir de la información del índice.
- **Comprobación de existencia**: comprueba la existencia de un elemento verificando si es -1 o no.

## 🧠 Ejemplo práctico de código 1: Búsqueda en una lista de tareas

Este es un ejemplo de búsqueda de la ubicación de una tarea con condiciones específicas a partir de una lista de tareas.

```ts
import { from, fromEvent } from 'rxjs';
import { findIndex } from 'rxjs';

interface Task {
  id: number;
  title: string;
  priority: 'high' | 'medium' | 'low';
  completed: boolean;
}

const tasks: Task[] = [
  { id: 1, title: 'Respuesta por correo electrónico', priority: 'low', completed: true },
  { id: 2, title: 'Preparación de documentos', priority: 'medium', completed: true },
  { id: 3, title: 'Preparación de reuniones', priority: 'high', completed: false },
  { id: 4, title: 'Revisión del código', priority: 'high', completed: false },
  { id: 5, title: 'Actualización de documentos', priority: 'low', completed: false }
];

// UICreación de
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Búsqueda de tareas';
container.appendChild(title);

// Visualización de la lista de tareas
const taskList = document.createElement('ul');
taskList.style.listStyle = 'none';
taskList.style.padding = '0';
tasks.forEach((task, index) => {
  const li = document.createElement('li');
  li.style.padding = '5px';
  li.style.borderBottom = '1px solid #eee';
  const status = task.completed ? '✅' : '⬜';
  const priorityBadge = task.priority === 'high' ? '🔴' : task.priority === 'medium' ? '🟡' : '🟢';
  li.textContent = `[${index}] ${status} ${priorityBadge} ${task.title}`;
  taskList.appendChild(li);
});
container.appendChild(taskList);

// Botón de búsqueda
const buttonContainer = document.createElement('div');
buttonContainer.style.marginTop = '10px';
container.appendChild(buttonContainer);

const button1 = document.createElement('button');
button1.textContent = 'Buscar la primera tarea no completada';
button1.style.marginRight = '10px';
buttonContainer.appendChild(button1);

const button2 = document.createElement('button');
button2.textContent = 'Buscar la primera tarea de alta prioridad';
buttonContainer.appendChild(button2);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
result.style.display = 'none';
container.appendChild(result);

// Buscar la primera tarea no completada
// NOTA.: Originalmente, el patrón recomendado era aplanar con switchMap El patrón recomendado es aplanar con
// En este caso, se da prioridad a la legibilidad subscribe anidada (en código de producción switchMap recomendado).
fromEvent(button1, 'click').subscribe(() => {
  // Anidado subscribe: Originalmente, el patrón recomendado era aplanar con switchMap Se recomienda aplanar con
  from(tasks).pipe(
    findIndex(task => !task.completed)
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ Encontrado en</strong><br>
        Posición: Índice ${index}<br>
        Tarea: ${task.title}<br>
        Prioridad: ${task.priority}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ Tarea inacabada no encontrada';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});

// Buscar la primera tarea de alta prioridad
// NOTA.: Originalmente, el patrón recomendado era aplanar con switchMap El patrón recomendado (en código de producción) es aplanar con switchMap recomendado).
fromEvent(button2, 'click').subscribe(() => {
  // Anidado subscribe: Originalmente, el patrón recomendado era aplanar con switchMap Se recomienda aplanar con
  from(tasks).pipe(
    findIndex(task => task.priority === 'high')
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ Encontrado en</strong><br>
        Posición: Índice ${index}<br>
        Tarea: ${task.title}<br>
        Estado de finalización: ${task.completed ? 'Completado' : 'Sin completar'}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ No se han encontrado tareas de alta prioridad';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});
```

- Encuentra la posición de la primera tarea en la lista de tareas que satisface la condición.
- Si no se encuentra, se devuelve `-1`.

## 🎯 Ejemplo práctico de código 2: Detección de la posición de los datos en tiempo real

Este ejemplo detecta la posición del primer valor del flujo que satisface la condición.

```ts
import { interval } from 'rxjs';
import { findIndex, map, take } from 'rxjs';

// UICreación de
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Búsqueda de datos en tiempo real';
container.appendChild(title);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = '50Búsqueda de posiciones con un valor mayor o igual a...';
container.appendChild(status);

const dataDisplay = document.createElement('div');
dataDisplay.style.marginTop = '10px';
dataDisplay.style.padding = '10px';
dataDisplay.style.border = '1px solid #ccc';
dataDisplay.style.maxHeight = '150px';
dataDisplay.style.overflow = 'auto';
container.appendChild(dataDisplay);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.fontWeight = 'bold';
container.appendChild(result);

// Generación de valores aleatorios (0~100)
const data$ = interval(500).pipe(
  take(20),
  map(i => ({ index: i, value: Math.floor(Math.random() * 100) }))
);

// Visualización de datos
data$.subscribe(data => {
  const div = document.createElement('div');
  const highlight = data.value >= 50 ? 'background-color: #fff9c4;' : '';
  div.style.cssText = `padding: 5px; ${highlight}`;
  div.textContent = `[${data.index}] Valor: ${data.value}`;
  dataDisplay.appendChild(div);
  dataDisplay.scrollTop = dataDisplay.scrollHeight;
});

// 50Buscar en el índice el primer valor de más de
data$.pipe(
  findIndex(data => data.value >= 50)
).subscribe(index => {
  status.textContent = '';
  if (index !== -1) {
    result.innerHTML = `
      ✅ 50Valor mayor o igual que encontrado<br>
      Posición: Índice ${index}
    `;
    result.style.color = 'green';
  } else {
    result.textContent = '❌ 50No se ha encontrado ningún valor mayor o igual que';
    result.style.color = 'orange';
  }
});
```

- Detecta la posición del primer valor por encima de 50 a partir de valores aleatorios generados cada 0,5 segundos.
- El resaltado se utiliza para mayor claridad visual.

## 🆚 Comparación con operadores similares

### findIndex vs find vs elementAt

```ts
import { from } from 'rxjs';
import { findIndex, find, elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// findIndex: Devuelve el índice del primer valor que cumple la condición
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);
// Salida.: 2Devuelve el índice del primer valor que cumple la condición30índice del primer par)

// find: Devuelve el primer valor que cumple la condición
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);
// Salida.: 30

// elementAt: Devuelve el valor en el índice especificado
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Salida.: 30
```

| Operador | Argumentos | Valor de retorno | Si no se encuentra. |
|---|---|---|---|
| `encontrarÍndice(predicado)` | Función condicional | Índice (numérico). | `-1` |
| Buscar(predicado)` | Función condicional | Valor propio | indefinido |
| elementoEn(índice)` | Índice | Valor propio | Error (sin valor por defecto) |

## 🔄 Comparación con Array.findIndex() de JavaScript

El método `findIndex` de RxJS se comporta de forma similar al método `Array.prototype.findIndex()` de JavaScript.

```ts
// JavaScript Matriz de
const numbers = [10, 20, 30, 40, 50];
const index1 = numbers.findIndex(n => n > 25);
console.log(index1); // 2

// RxJS (devuelve el primer valor en el índice especificado que satisface la condición) Observable
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log); // 2
```

**Principales diferencias**.
- **Array**: devuelve el resultado de forma sincrónica e inmediata.
- **Observable**: asíncrono, espera a que los valores fluyan desde el stream.

## ⚠️ Notas.

### 1. devuelve -1 si no se encuentra

Si ningún valor satisface la condición, devuelve `-1` en lugar de un error.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 9]);

numbers$.pipe(
  findIndex(n => n > 10)
).subscribe(index => {
  if (index === -1) {
    console.log('No se ha encontrado ningún valor que cumpla la condición');
  } else {
    console.log(`Índice: ${index}`);
  }
});
// Salida.: No se ha encontrado ningún valor que cumpla la condición
```

### 2. completa cuando se encuentra por primera vez.

El flujo se completa en cuanto se encuentra el primer valor que satisface la condición.

```ts
import { interval } from 'rxjs';
import { findIndex, tap } from 'rxjs';

interval(1000).pipe(
  tap(val => console.log(`Valor: ${val}`)),
  findIndex(n => n >= 3)
).subscribe(index => {
  console.log(`Índice: ${index}`);
});
// Salida.:
// Valor: 0
// Valor: 1
// Valor: 2
// Valor: 3
// Índice: 3
```

### 3. Seguridad tipográfica en TypeScript

`findIndex` siempre devuelve el tipo `number`.

```ts
import { Observable, from } from 'rxjs';
import { findIndex } from 'rxjs';

interface User {
  id: number;
  name: string;
  isActive: boolean;
}

function findFirstInactiveUserIndex(
  users$: Observable<User>
): Observable<number> {
  return users$.pipe(
    findIndex(user => !user.isActive)
  );
}

const users$ = from([
  { id: 1, name: 'Alice', isActive: true },
  { id: 2, name: 'Bob', isActive: false },
  { id: 3, name: 'Charlie', isActive: true }
]);

findFirstInactiveUserIndex(users$).subscribe(index => {
  // index es una matriz de number tipo
  if (index !== -1) {
    console.log(`El primer usuario inactivo es el índice ${index} es`);
  }
});
// Salida.: El primer usuario inactivo es el índice 1 es
```

### 4. el índice empieza en 0

Al igual que con las matrices, los índices empiezan en 0.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const items$ = from(['A', 'B', 'C', 'D']);

items$.pipe(
  findIndex(item => item === 'A')
).subscribe(console.log);
// Salida.: 0(primer elemento)
```

## 📚 Operadores relacionados.

- **[find](. /find)** - Obtiene el primer valor que satisface la condición.
- **[elementoEn](. /elementAt)** - Obtiene el valor en el índice especificado.
- **[first](. /primero)** - Obtener el primer valor.
- **[filtro](. /filtro)** - Obtener todos los valores que cumplan la condición.

## Resumen.

El operador `findIndex` devuelve el índice del primer valor que satisface la condición.

- ✅ Comportamiento similar al de `Array.findIndex()` de JavaScript.
- ✅ Ideal cuando se necesita información sobre el índice.
- ✅ Devuelve `-1` si no se encuentra (no es un error)
- ✅ Finaliza inmediatamente si se encuentra
- ⚠️ El valor de retorno es siempre de tipo `number` (-1 o un entero mayor o igual que 0)
- ⚠️ Utilice `find` si se requiere el valor en sí mismo
