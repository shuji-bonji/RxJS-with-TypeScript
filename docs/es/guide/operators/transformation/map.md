---
description: "El operador map es un medio básico de transformación que aplica una función a cada valor en un Observable para generar nuevos valores. Se usa ampliamente para formateo de formularios, procesamiento de respuestas API y transformación de datos. Explicamos la inferencia de tipos de TypeScript, combinación con otros operadores y optimización del rendimiento."
---

# map - Aplicar función de transformación a cada valor

El operador `map` aplica una función especificada a **cada valor** en el stream y genera nuevos valores transformados.
Es similar al método `Array.prototype.map` de arrays, pero opera sobre **streams asíncronos**.


## 🔰 Sintaxis básica y uso

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3).pipe(
  map(value => value * 10)
).subscribe(console.log);
// Salida: 10, 20, 30
```

Aplica la función value => value * 10 a cada valor y genera nuevos valores.

[🌐 Documentación Oficial RxJS - map](https://rxjs.dev/api/index/function/map)


## 💡 Patrones de uso típicos
- Transformación de respuestas API (extraer solo las propiedades necesarias)
- Formateo de datos de entrada de formularios
- Procesamiento de números y cadenas en el stream
- Extraer solo los datos necesarios de eventos de UI


## 🧠 Ejemplo de código práctico (con UI)

Ejemplo que duplica un número ingresado y lo muestra en tiempo real.

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Crear campo de entrada
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Ingrese un número';
document.body.appendChild(input);

// Crear campo de salida
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Stream de eventos de entrada
fromEvent(input, 'input').pipe(
  map(event => Number((event.target as HTMLInputElement).value)),
  map(value => value * 2)
).subscribe(result => {
  output.textContent = `Valor duplicado: ${result}`;
});
```

- El valor de entrada se duplica y se muestra en tiempo real.
- Al aplicar map consecutivamente, se logra una cadena simple de transformación de datos.
