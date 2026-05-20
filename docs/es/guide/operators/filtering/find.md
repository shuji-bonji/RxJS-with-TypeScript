---
description: "find es un operador de filtrado de RxJS que encuentra el primer valor que satisface una condición y lo emite, completando el flujo inmediatamente. Es ideal para situaciones en las que se desea encontrar un elemento específico de una matriz o lista, como la búsqueda de usuarios, la comprobación del inventario o la detección de registros de errores. Si no se encuentra ningún valor, devuelve undefined y en TypeScript el valor de retorno es de tipo T | undefined."
---

# find - encuentra el primer valor que satisface la condición

El operador `find` encuentra y emite el **primer valor que satisface la condición** y completa el flujo inmediatamente. Si no encuentra ningún valor, devuelve `undefined`.

## 🔰 Sintaxis básica y uso

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  find(n => n % 2 === 0)
).subscribe(console.log);
// Salida.: 8(primer número par)
```

**Flujo de operación**:.
1. comprobar 1, 3, 5, 7 → condición no cumplida.
2. comprobación 8 → condición cumplida → salida 8 y completa
3. 9, 10 no evaluados

[🌐 Documentación oficial de RxJS - `find`](https://rxjs.dev/api/operators/find)

## 🆚 Contraste con first

`find` y `first` son similares, pero su uso es diferente.

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// first: Primer valor que cumple la condición (la condición es opcional)
numbers$.pipe(
  first(n => n > 5)
).subscribe(console.log);
// Salida.: 7

// find: Primer valor que cumple la condición (la condición es obligatoria)
numbers$.pipe(
  find(n => n > 5)
).subscribe(console.log);
// Salida.: 7
```

| Operador. | Especificación de la condición | Si no se encuentra ningún valor | Caso de uso. |
|---|---|---|---|
| first()` (primero) | Opción | Error (`EmptyError`) | Obtener el primer valor |
| first(predicado)` | Opcional | Error (`EmptyError`) | Obtener condicional. |
| Encontrar(predicado)` | Obligatorio. | Salida `undefined`. | Búsqueda y comprobación de existencia |

## 💡 Patrón típico de utilización

1. **Búsqueda de usuarios**.

```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface User {
     id: number;
     name: string;
     email: string;
   }

   const users$ = from([
     { id: 1, name: 'Alice', email: 'alice@example.com' },
     { id: 2, name: 'Bob', email: 'bob@example.com' },
     { id: 3, name: 'Charlie', email: 'charlie@example.com' }
   ] as User[]);

   // ID(la condición es opcional)2Buscar usuarios con
   users$.pipe(
     find(user => user.id === 2)
   ).subscribe(user => {
     if (user) {
       console.log(`Encontrado: ${user.name}`);
     } else {
       console.log('Usuario no encontrado');
     }
   });
   // Salida.: Encontrado: Bob
   ```

2. **Comprobación de inventario**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface Product {
     id: string;
     name: string;
     stock: number;
   }

   const products$ = from([
     { id: 'A1', name: 'PortátilPC', stock: 0 },
     { id: 'A2', name: 'Ratón', stock: 15 },
     { id: 'A3', name: 'Teclados', stock: 8 }
   ] as Product[]);

   // Averigüe lo que está agotado
   products$.pipe(
     find(product => product.stock === 0)
   ).subscribe(product => {
     if (product) {
       console.log(`Agotado: ${product.name}`);
     } else {
       console.log('Todo en stock');
     }
   });
   // Salida.: Agotado: PortátilPC
   ```

3. **Buscar registro de errores**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App started' },
     { timestamp: 2, level: 'info' as const, message: 'User logged in' },
     { timestamp: 3, level: 'error' as const, message: 'Connection failed' },
     { timestamp: 4, level: 'info' as const, message: 'Retry successful' }
   ] as LogEntry[]);

   // Buscar el primer error
   logs$.pipe(
     find(log => log.level === 'error')
   ).subscribe(log => {
     if (log) {
       console.log(`Detección de errores: ${log.message} (Hora: ${log.timestamp})`);
     }
   });
   // Salida.: Detección de errores: Connection failed (Hora: 3)
   ```

## 🧠 Ejemplo práctico de código (búsqueda de productos)

Este es un ejemplo de búsqueda de productos que coincidan con criterios específicos del stock.

```

ts.
import { from, fromEvent } from 'rxjs';
import { find } from 'rxjs';

interfaz Producto {
  id: cadena;
  nombre: string
  precio: número
  categoría: cadena;
}

const productos: Producto[] = [
  { id: 'P1', nombre: 'Ratón inalámbrico', precio: 2980, categoría: 'Periféricos PC' }
  { id: 'P2', name: 'Teclado mecánico', price: 8980, category: 'Periféricos PC' }
  { id: 'P3', name: 'Memoria USB 64GB', price: 1480, category: 'Almacenamiento' }
  { id: 'P4', name: 'Monitor 27-inch', price: 29800, category: 'Displays' }
  { id: 'P5', name: 'laptop stand', price: 3980, category: 'PC peripherals' }
];

// Creación de elementos de interfaz
const container = document.createElement('div');.
document.body.appendChild(contenedor);

const title = document.createElement('h3');
title.textContent = 'Búsqueda de productos';
container.appendChild(title);

const input = document.createElement('input');
input.type = 'número';
input.placeholder = 'Introduzca el precio máximo';
input.style.marginRight = '10px';
container.appendChild(input);

const searchButton = document.createElement('button');
searchButton.textContent = 'buscar';
container.appendChild(searchButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
container.appendChild(resultado);

// Proceso de búsqueda
// Nota: aunque originalmente el patrón recomendado es aplanar con un switchMap, pero,
// Nota: aunque el patrón recomendado es aplanar con un switchMap, // aquí anidamos el subscribe por legibilidad, // porque incluye validación UI (early return).
// Considera una implementación plana usando switchMap en código de producción.
fromEvent(searchButton, 'click').subscribe(() => {
  const maxPrice = parseInt(input.value);.

  if (isNaN(maxPrice)) {
    result.textContent = 'Por favor, introduzca un precio';
    result.style.colour = 'rojo';
    return;
  }

  // Nest subscribe: originalmente se recomendaba aplanar con switchMap
  from(productos).pipe(
    find(producto => producto.precio <= preciomáximo)
  ).subscribe(producto => {
    if (producto) {
      result.innerHTML = `
        <strong>¡Encontrado! </strong><br>
        Nombre del producto: ${product.name}<br>
        Precio: ${product.price.toLocaleString()}<br>
        Categoría: ${product.category}
      `;
      result.style.color = 'verde';
    } else {
      result.textContent = `¥{maxPrice.toLocaleString()} o menos producto no encontrado `;
      result.style.color = 'orange'; }
    }
  });
});

```

Este código busca y muestra el primer producto por debajo del precio introducido por el usuario.

## 🎯 filter La diferencia entre

`find` y `filter` se utilizan con fines distintos.

```

ts.
import { from } from 'rxjs';
import { find, filter } from 'rxjs';

const números$ = from([1, 3, 5, 7, 8, 9, 10]);

// filter: muestra todos los valores que cumplen la condición
números$.pipe(
  filter(n => n > 5)
).subscribe({
  next: console.log,.
  complete: () => console.log('filtro complete')
});
// Salida: 7, 8, 9, 10, filter complete

// find: salida sólo el primer valor que coincida con la condición
números$.pipe(
  find(n => n > 5)
).subscribe({
  next: console.log,.
  complete: () => console.log('find complete')
});
// salida: 7, find completo

```

| Operador | Número de salidas | Tiempo de realización | Caso de uso |
|---|---|---|---|
| `filter(predicate)` | Todos los valores que cumplen la condición | Al finalizar el flujo original | Depuración de datos |
| `find(predicate)` | Sólo el primer valor que cumpla los criterios | Inmediatamente después del descubrimiento | Búsqueda y comprobación de existencia |

## 📋 Uso seguro

TypeScript Este es un ejemplo de implementación segura que utiliza genéricos en

```

ts.
import { Observable, from } from 'rxjs';
import { find } from 'rxjs';

interfaz Task {
  id: número;
  título: string
  completed: booleano;
  prioridad: 'alta' | 'media' | 'baja'; }
}

function findTaskById(
  tareas$: Observable\<Tarea>,.
  id: número
): Observable | undefined> {
  return tareas$.pipe(
    find(tarea => tarea.id === id)
  );
}

función findFirstIncompleteTask(
  tareas$: Observable\<Tarea>
): Observable<Tarea | undefined> {
  return tareas$.pipe(
    find(tarea => !tarea.complete)
  );
}

// Ejemplo de uso
const tareas$ = from([.
  { id: 1, title: 'Tarea A', completed: true, priority: 'high' as const }
  { id: 2, title: 'Tarea B', complete: false, prioridad: 'media' as const }
  { id: 3, title: 'Tarea C', completada: false, prioridad: 'baja' as const }
] as Task[]);.

// Búsqueda por ID
findTaskById(tareas$, 2).subscribe(tarea => {
  if (tarea) {
    console.log(`encontrado: ${tarea.titulo}`);
  } else {
    console.log('Tarea no encontrada'); }
  }
});
// Salida: encontrada: tarea B

// Encontrar tareas no completadas
findFirstIncompleteTask(tareas$).subscribe(tarea => {
  if (tarea) {
    console.log(`Siguiente tarea: ${task.title} (prioridad: ${task.priority})`);
  }
});
// Salida: siguiente tarea: tarea B (prioridad: media)

```

## 🔄 find y findIndex La diferencia entre

RxJSen los operadores `findIndex` también están disponibles.

```

ts
import { from } from 'rxjs';
import { find, findIndex } from 'rxjs';

const números$ = from([10, 20, 30, 40, 50]);

// find: devuelve un valor
números$.pipe(
  find(n => n > 25)
).subscribe(console.log);.
// salida: 30

// findIndex: devuelve el índice
números$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);.
// Salida: 2 (índice de 30)

```

| Operador | Devuelve el valor | si no se encuentra el valor |
|---|---|---|
| `find(predicate)` | Valor propiamente dicho | `undefined` |
| `findIndex(predicate)` | Índice (valor numérico) | `-1` |

## ⚠️ Un error común

> [!NOTE]
> `find` si no se encuentra el valor `undefined` es emitido. Esto no da lugar a un error. Si se requiere un error, utilice `first` para ser utilizado.

### Error.: Tratamiento del error esperado si no se encuentra el valor.

```

ts.
import { from } from 'rxjs';
import { find } from 'rxjs';

const números$ = from([1, 3, 5, 7]);

// ❌ Mal ejemplo: gestión de errores esperada pero no invocada
números$.pipe(
  find(n => n > 10)
).subscribe({
  next: console.log,.
  error: err => console.log('Error:', err) // no llamado
});
// salida: indefinida

```

### Positivo: undefined Comprobar o first utilizar el

```

ts.
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const números$ = from([1, 3, 5, 7]);

// ✅ Buen ejemplo 1: comprobación de indefinidos
números$.pipe(
  find(n => n > 10)
).subscribe(result => {
  ¡if (resultado ! == undefined) {
    console.log('Encontrado:', resultado);
  } else {
    console.log('No encontrado:'); }
  }
});
// Salida: no encontrado

// ✅ Buen ejemplo 2: usa first si necesitas un error
numbers$.pipe(
  first(n => n > 10, 0) // especificar valor por defecto
).subscribe({
  next: console.log,.
  error: err => console.log('Error:', err.message)
});
// Salida: 0
```

## 🎓 Resumen

### Cuándo debes usar find.
- ✅ Cuando se quiere encontrar el primer valor que satisface una condición.
- ✅ Cuando quieras comprobar la existencia de un valor.
- ✅ Cuando se desea tratar un valor como `undefined` si no se encuentra.
- ✅ Cuando se desea encontrar un elemento concreto en una matriz o lista.

### Cuando debe utilizar first.
- ✅ Si quieres obtener el primer valor.
- ✅ Si desea emitir un error si no se encuentra el valor

### ¿Cuándo se debe utilizar filter?
- ✅ Si necesita todos los valores que coincidan con una condición
- ✅ Si desea filtrar los datos

### Notas.
- ⚠️ `find` da como resultado `undefined` si no se encuentra (no es un error)
- ⚠️ Completa inmediatamente con el primer valor que satisface la condición
- ⚠️ TypeScript devuelve un valor de tipo `T | undefined`.

## 🚀 Siguiente paso.

- **[first](./first)** - aprende a obtener el primer valor.
- **[filter](./filter)** - aprende a filtrar en base a condiciones.
- findIndex](https://rxjs.dev/api/operators/findIndex)** - aprende a obtener el índice del primer valor que satisface una condición (documentación oficial).
- **[filtro-operador-casos-prácticos](./practical-use-cases)** - aprende casos de uso reales
