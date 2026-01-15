---
description: El operador distinctUntilKeyChanged se enfoca en una propiedad específica dentro de un flujo de objetos y emite solo cuando ese valor difiere del anterior. Omite eficientemente datos duplicados consecutivos y es útil para detectar cambios de estado y optimizar actualizaciones de listas.
titleTemplate: ':title'
---

# distinctUntilKeyChanged - Detectar Cambio de Clave

El operador `distinctUntilKeyChanged` se enfoca en una clave específica (propiedad) de un objeto y emite solo cuando ese valor difiere del anterior.
Es útil para omitir eficientemente duplicados consecutivos.


## 🔰 Sintaxis Básica y Uso

```ts
import { from } from 'rxjs';
import { distinctUntilKeyChanged } from 'rxjs';

const users = [
  { id: 1, name: 'Tanaka' },
  { id: 2, name: 'Tanaka' }, // Mismo nombre, omitir
  { id: 3, name: 'Sato' },
  { id: 4, name: 'Suzuki' },
  { id: 5, name: 'Suzuki' }, // Mismo nombre, omitir
  { id: 6, name: 'Tanaka' }
];

from(users).pipe(
  distinctUntilKeyChanged('name')
).subscribe(console.log);

// Salida:
// { id: 1, name: 'Tanaka' }
// { id: 3, name: 'Sato' }
// { id: 4, name: 'Suzuki' }
// { id: 6, name: 'Tanaka' }
```

- Emite solo cuando el valor de la propiedad especificada `name` cambia.
- Otras propiedades (por ejemplo, `id`) no se comparan.

[🌐 Documentación Oficial de RxJS - `distinctUntilKeyChanged`](https://rxjs.dev/api/operators/distinctUntilKeyChanged)


## 💡 Patrones de Uso Típicos

- Actualizar visualización de lista solo cuando una propiedad específica cambia
- Detectar solo cambios en atributos específicos en flujos de eventos
- Controlar eliminación de duplicados basándose en una clave


## 🧠 Ejemplo de Código Práctico (con UI)

Ingrese un nombre en el cuadro de texto y presione Enter para registrarlo.
**Si se ingresa el mismo nombre consecutivamente, se ignora**, y se agrega a la lista solo cuando se ingresa un nombre diferente.

```ts
import { fromEvent } from 'rxjs';
import { map, filter, scan, distinctUntilKeyChanged } from 'rxjs';

// Crear área de salida
const output = document.createElement('div');
document.body.appendChild(output);

const title = document.createElement('h3');
title.textContent = 'Ejemplo Práctico de distinctUntilKeyChanged';
output.appendChild(title);

// Formulario de entrada
const input = document.createElement('input');
input.placeholder = 'Ingrese nombre y presione Enter';
document.body.appendChild(input);

// Flujo de evento de entrada
fromEvent<KeyboardEvent>(input, 'keydown').pipe(
  filter((e) => e.key === 'Enter'),
  map(() => input.value.trim()),
  filter((name) => name.length > 0),
  scan((_, name, index) => ({ id: index + 1, name }), { id: 0, name: '' }),
  distinctUntilKeyChanged('name')
).subscribe((user) => {
  const item = document.createElement('div');
  item.textContent = `Entrada de usuario: ID=${user.id}, Nombre=${user.name}`;
  output.appendChild(item);
});
```

- Si se ingresa el mismo nombre consecutivamente, se omite.
- Se muestra solo cuando se ingresa un nombre nuevo.
