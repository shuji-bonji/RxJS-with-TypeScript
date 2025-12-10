---
description: "El operador groupBy agrupa los valores del stream según una clave especificada y crea Observables separados para cada grupo. Explica agregación por categoría, procesamiento por usuario, clasificación de datos y casos de uso prácticos con implementación segura en TypeScript."
---

# groupBy - Agrupar valores según clave

El operador `groupBy` **agrupa los valores emitidos del stream según una clave especificada** y genera cada grupo como un Observable individual.
Es útil al clasificar datos por categoría o aplicar diferentes procesamientos a cada grupo.

## 🔰 Sintaxis básica y uso

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray, map } from 'rxjs';

interface Person {
  name: string;
  age: number;
}

const people: Person[] = [
  { name: 'Taro', age: 25 },
  { name: 'Hanako', age: 30 },
  { name: 'Jiro', age: 25 },
  { name: 'Misaki', age: 30 },
  { name: 'Kenta', age: 35 },
];

from(people).pipe(
  groupBy(person => person.age), // Agrupar por edad
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(arr => ({ age: group.key, people: arr }))
    )
  )
).subscribe(result => {
  console.log(`Edad ${result.age}:`, result.people);
});

// Salida:
// Edad 25: [{name: 'Taro', age: 25}, {name: 'Jiro', age: 25}]
// Edad 30: [{name: 'Hanako', age: 30}, {name: 'Misaki', age: 30}]
// Edad 35: [{name: 'Kenta', age: 35}]
```

- Agrupar usando `groupBy(person => person.age)` con la edad como clave
- Cada grupo se trata como `GroupedObservable`, con acceso a la clave del grupo mediante la propiedad `key`
- Procesar el Observable de cada grupo con `mergeMap`

[🌐 Documentación oficial de RxJS - `groupBy`](https://rxjs.dev/api/operators/groupBy)

## 💡 Patrones de uso típicos

- Clasificación de datos por categoría
- Procesamiento de agregación por grupo
- Procesamiento por tipo de log o evento
- Agrupación y transformación de datos

## 🧠 Ejemplo de código práctico (con UI)

Ejemplo que agrupa por color y muestra la cantidad al hacer clic en un botón.

```ts
import { fromEvent, from } from 'rxjs';
import { groupBy, mergeMap, toArray, switchMap, map } from 'rxjs';

// Crear botones
const colors = ['rojo', 'azul', 'verde', 'amarillo'];
colors.forEach(color => {
  const button = document.createElement('button');
  button.textContent = color;
  button.style.margin = '5px';
  button.style.padding = '10px';
  button.dataset.color = color;
  document.body.appendChild(button);
});

const calculateButton = document.createElement('button');
calculateButton.textContent = 'Calcular';
calculateButton.style.margin = '5px';
calculateButton.style.padding = '10px';
document.body.appendChild(calculateButton);

// Crear área de salida
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.fontFamily = 'monospace';
document.body.appendChild(output);

// Registrar colores en los que se hizo clic
const clicks: string[] = [];

// Evento de clic en botones de color
fromEvent(document, 'click').subscribe((event: Event) => {
  const target = event.target as HTMLElement;
  const color = target.dataset.color;
  if (color) {
    clicks.push(color);
    output.innerHTML = `Colores seleccionados: ${clicks.join(', ')}`;
  }
});

// Agrupar y mostrar al hacer clic en el botón de cálculo
fromEvent(calculateButton, 'click').pipe(
  switchMap(() =>
    from(clicks).pipe(
      groupBy(color => color),
      mergeMap(group =>
        group.pipe(
          toArray(),
          map(items => ({ color: group.key, count: items.length }))
        )
      ),
      toArray()
    )
  )
).subscribe(results => {
  if (results.length === 0) {
    output.innerHTML = '<p>Aún no se han seleccionado colores</p>';
    return;
  }
  const resultText = results
    .map(r => `${r.color}: ${r.count} veces`)
    .join('<br>');
  output.innerHTML = `<h3>Resultados del cálculo</h3>${resultText}`;
});
```

- Hacer clic en los botones de color para seleccionar colores
- Agrupar por color con el botón "Calcular" y mostrar la cantidad
- Agrupar por color con `groupBy` y contar los elementos de cada grupo

## 🎯 Ejemplo de agregación por categoría

Ejemplo que clasifica productos por categoría y calcula el total por categoría.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, reduce, map } from 'rxjs';

interface Product {
  name: string;
  category: string;
  price: number;
}

const products: Product[] = [
  { name: 'manzana', category: 'frutas', price: 150 },
  { name: 'naranja', category: 'frutas', price: 100 },
  { name: 'zanahoria', category: 'verduras', price: 80 },
  { name: 'tomate', category: 'verduras', price: 120 },
  { name: 'leche', category: 'lácteos', price: 200 },
  { name: 'queso', category: 'lácteos', price: 300 },
];

from(products).pipe(
  groupBy(product => product.category),
  mergeMap(group =>
    group.pipe(
      reduce((total, product) => total + product.price, 0),
      map(total => ({ category: group.key, total }))
    )
  )
).subscribe(result => {
  console.log(`${result.category}: ${result.total} yenes`);
});

// Salida:
// frutas: 250 yenes
// verduras: 200 yenes
// lácteos: 500 yenes
```

## 🎯 Ejemplo de uso del selector de elementos

También se pueden transformar valores al agrupar.

```ts
import { from } from 'rxjs';
import { groupBy, map, mergeMap, toArray } from 'rxjs';

interface Student {
  name: string;
  grade: number;
  score: number;
}

const students: Student[] = [
  { name: 'Taro', grade: 1, score: 85 },
  { name: 'Hanako', grade: 2, score: 92 },
  { name: 'Jiro', grade: 1, score: 78 },
  { name: 'Misaki', grade: 2, score: 88 },
];

from(students).pipe(
  groupBy(
    student => student.grade,           // Selector de clave
    student => student.name             // Selector de elemento (mantener solo el nombre)
  ),
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(names => ({ grade: group.key, students: names }))
    )
  )
).subscribe(result => {
  console.log(`Grado ${result.grade}:`, result.students.join(', '));
});

// Salida:
// Grado 1: Taro, Jiro
// Grado 2: Hanako, Misaki
```

- Primer argumento: selector de clave (criterio de agrupación)
- Segundo argumento: selector de elemento (valor a guardar en el grupo)

## 🎯 Uso de groupBy con seguridad de tipos

Ejemplo aprovechando la inferencia de tipos de TypeScript.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray, map } from 'rxjs';

type LogLevel = 'info' | 'warning' | 'error';

interface LogEntry {
  level: LogLevel;
  message: string;
  timestamp: number;
}

const logs: LogEntry[] = [
  { level: 'info', message: 'Inicio de aplicación', timestamp: 1000 },
  { level: 'warning', message: 'Mensaje de advertencia', timestamp: 2000 },
  { level: 'error', message: 'Error ocurrido', timestamp: 3000 },
  { level: 'info', message: 'Procesamiento completo', timestamp: 4000 },
  { level: 'error', message: 'Error de conexión', timestamp: 5000 },
];

from(logs).pipe(
  groupBy(log => log.level),
  mergeMap(group =>
    group.pipe(
      toArray(),
      map(entries => ({
        level: group.key,
        count: entries.length,
        messages: entries.map(e => e.message)
      }))
    )
  )
).subscribe(result => {
  console.log(`[${result.level.toUpperCase()}] ${result.count} elementos`);
  result.messages.forEach(msg => console.log(`  - ${msg}`));
});

// Salida:
// [INFO] 2 elementos
//   - Inicio de aplicación
//   - Procesamiento completo
// [WARNING] 1 elemento
//   - Mensaje de advertencia
// [ERROR] 2 elementos
//   - Error ocurrido
//   - Error de conexión
```

## 🎯 Aplicar diferentes procesamientos a cada grupo

Ejemplo aplicando diferentes procesamientos a cada grupo.

```ts
import { from, of } from 'rxjs';
import { groupBy, mergeMap, delay, map } from 'rxjs';

interface Task {
  id: number;
  priority: 'high' | 'medium' | 'low';
  name: string;
}

const tasks: Task[] = [
  { id: 1, priority: 'high', name: 'Tarea urgente' },
  { id: 2, priority: 'low', name: 'Tarea para después' },
  { id: 3, priority: 'high', name: 'Tarea importante' },
  { id: 4, priority: 'medium', name: 'Tarea normal' },
];

from(tasks).pipe(
  groupBy(task => task.priority),
  mergeMap(group => {
    // Establecer tiempo de retardo según prioridad
    const delayTime =
      group.key === 'high' ? 0 :
      group.key === 'medium' ? 1000 :
      2000;

    return group.pipe(
      delay(delayTime),
      map(task => ({ ...task, processedAt: Date.now() }))
    );
  })
).subscribe(task => {
  console.log(`[${task.priority}] Procesar ${task.name}`);
});

// Salida (en orden de prioridad):
// [high] Procesar Tarea urgente
// [high] Procesar Tarea importante
// (después de 1 segundo)
// [medium] Procesar Tarea normal
// (después de 1 segundo adicional)
// [low] Procesar Tarea para después
```

## ⚠️ Puntos de atención

### Gestión de suscripción de Observables de grupo

`groupBy` crea un Observable para cada grupo. Estos Observables pueden causar fugas de memoria si no se suscriben adecuadamente.

```ts
// ❌ Mal ejemplo: no suscribirse a los Observables de grupo
from([1, 2, 3, 4, 5]).pipe(
  groupBy(n => n % 2 === 0 ? 'even' : 'odd')
).subscribe(group => {
  // No se suscribe al Observable del grupo
  console.log('Grupo:', group.key);
});
```

**Solución**: Asegurarse de procesar cada grupo con `mergeMap`, `concatMap`, `switchMap`, etc.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray } from 'rxjs';

// ✅ Buen ejemplo: procesar cada grupo adecuadamente
from([1, 2, 3, 4, 5]).pipe(
  groupBy(n => n % 2 === 0 ? 'even' : 'odd'),
  mergeMap(group =>
    group.pipe(toArray())
  )
).subscribe(console.log);
```

### Generación dinámica de grupos

Se crea un nuevo Observable de grupo cada vez que aparece una nueva clave. Se requiere precaución cuando hay muchos tipos de claves.

```ts
// Ejemplo donde los tipos de claves pueden aumentar infinitamente
fromEvent(document, 'click').pipe(
  groupBy(() => Math.random()) // Clave diferente cada vez
).subscribe(); // Peligro de fuga de memoria
```

## 📚 Operadores relacionados

- [`partition`](https://rxjs.dev/api/index/function/partition) - Dividir en dos Observables según condición
- [`reduce`](./reduce) - Obtener resultado final de agregación
- [`scan`](./scan) - Agregación acumulativa
- [`toArray`](../utility/toArray) - Reunir todos los valores en un array

## Resumen

El operador `groupBy` agrupa los valores del stream según una clave y puede **tratar cada grupo como un Observable individual**. Es muy útil para el procesamiento complejo de datos como clasificación de datos, agregación por categoría y procesamiento diferente por grupo. Sin embargo, cada Observable de grupo debe suscribirse adecuadamente, y normalmente se usa en combinación con `mergeMap` u otros operadores.
