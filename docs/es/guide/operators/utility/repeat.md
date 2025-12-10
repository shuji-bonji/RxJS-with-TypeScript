---
description: El operador repeat vuelve a ejecutar todo el stream un número especificado de veces después de que el Observable fuente se completa exitosamente. Se puede usar para polling periódico, animación repetitiva y otras situaciones que requieren un control diferente al de retry.
---

# repeat - Repetir Stream

El operador `repeat` vuelve a ejecutar todo el stream un número especificado de veces después de que el Observable fuente se ha **completado exitosamente**.
Esto es útil para procesos de polling, animaciones repetidas y controles que son diferentes de los reintentos.

## 🔰 Sintaxis Básica y Operación

El uso más simple es configurar una secuencia de valores para que se repita cierto número de veces.

```ts
import { of } from 'rxjs';
import { repeat } from 'rxjs';

of('A', 'B')
  .pipe(
    repeat(2) // Repetir toda la secuencia 2 veces (salida 2 veces en total)
  )
  .subscribe(console.log);
// Salida:
// A
// B
// A
// B
```

[🌐 Documentación Oficial de RxJS - repeat](https://rxjs.dev/api/index/function/repeat)

## 💡 Ejemplo de Uso Típico

Por ejemplo, se usa para procesos simples de polling o animaciones de visualización repetidas.

```ts
import { of } from 'rxjs';
import { tap, delay, repeat } from 'rxjs';

of('✅ Datos recuperados exitosamente')
  .pipe(
    tap(() => console.log('Solicitud iniciada')),
    delay(1000),
    repeat(3) // Repetir 3 veces
  )
  .subscribe(console.log);
// Salida:
// Solicitud iniciada
// ✅ Datos recuperados exitosamente
// main.ts:6 Solicitud iniciada
// ✅ Datos recuperados exitosamente
// main.ts:6 Solicitud iniciada
// ✅ Datos recuperados exitosamente
```

En este ejemplo, "solicitud → recuperación de datos" se repite tres veces cada segundo.

## 🧪 Ejemplo de Código Práctico (con UI)

```ts
import { of } from 'rxjs';
import { repeat, tap } from 'rxjs';

// Área de visualización de salida
const repeatOutput = document.createElement('div');
repeatOutput.innerHTML = '<h3>Ejemplo de repeat:</h3>';
document.body.appendChild(repeatOutput);

// Visualización de contador de repetición
let repeatCount = 0;
const repeatCountDisplay = document.createElement('div');
repeatCountDisplay.textContent = `Contador de repetición: ${repeatCount}`;
repeatCountDisplay.style.fontWeight = 'bold';
repeatOutput.appendChild(repeatCountDisplay);

// Área de salida de valores
const valuesOutput = document.createElement('div');
valuesOutput.style.marginTop = '10px';
valuesOutput.style.padding = '10px';
valuesOutput.style.border = '1px solid #ddd';
valuesOutput.style.maxHeight = '200px';
valuesOutput.style.overflowY = 'auto';
repeatOutput.appendChild(valuesOutput);

// Repetición de secuencia
of('A', 'B', 'C')
  .pipe(
    tap(() => {
      repeatCount++;
      repeatCountDisplay.textContent = `Contador de repetición: ${repeatCount}`;
    }),
    repeat(3)
  )
  .subscribe((val) => {
    const valueItem = document.createElement('div');
    valueItem.textContent = `Valor: ${val} (repetición ${repeatCount})`;
    valuesOutput.appendChild(valueItem);
  });

```

## ✅ Resumen

- `repeat` **vuelve a ejecutar todo el Observable después de completarse exitosamente**
- A diferencia de `retry`, **no se vuelve a ejecutar en caso de error**
- Se puede usar para animaciones repetitivas, como procesos de polling y **marcadores de posición parpadeantes**
