---
description: El operador tap es un operador de utilidad que permite ejecutar efectos secundarios sin afectar el valor del stream. Ideal para depuración con salida de log, control de estados de carga, seguimiento de análisis, monitoreo de errores y otras aplicaciones donde se realiza procesamiento externo mientras se observa el stream. Los efectos secundarios se pueden gestionar en código declarativo mientras se mantiene la seguridad de tipos de TypeScript.
---

# tap - Ejecutar Efectos Secundarios

El operador `tap` se utiliza para "ejecutar efectos secundarios sin modificar el stream."
Ideal para logging, depuración u otras operaciones que no afectan los valores.

## 🔰 Sintaxis Básica y Operación

Utilizado en situaciones donde desea agregar solo efectos secundarios sin cambiar el flujo de valores.

```ts
import { of, tap } from 'rxjs';

of(42).pipe(
  tap(value => console.log('tap:', value))
).subscribe();
// Salida:
// tap: 42
```

En este ejemplo, el valor emitido desde `of(42)` se registra a medida que pasa por `tap`.
Debido a que `tap` pasa el valor "tal cual", no tiene efecto en el contenido del stream.

[🌐 Documentación Oficial de RxJS - tap](https://rxjs.dev/api/index/function/tap)

## 💡 Casos de Uso Típicos

`tap` a menudo se utiliza para los siguientes propósitos:

- Depuración y logging
- Alternar estado de carga
- Mostrar notificaciones toast
- Activar actualizaciones de UI

```ts
import { of, tap, map } from 'rxjs';

of(Math.random()).pipe(
  tap(val => console.log('Valor recuperado:', val)),
  map(n => n > 0.5 ? 'Alto' : 'Bajo'),
  tap(label => console.log('Etiqueta:', label))
).subscribe();
// Salida:
// Valor recuperado: 0.09909888881113504
// Etiqueta: Bajo
```


## 🧪 Ejemplo de Código Práctico (con UI)

El siguiente es un ejemplo de agregar logs al DOM usando tap.

```ts
import { of } from 'rxjs';
import { tap, map } from 'rxjs';

// Elemento para salida de log
const logOutput = document.createElement('div');
document.body.appendChild(logOutput);

// Secuencia de valores
of(1, 2, 3, 4, 5)
  .pipe(
    tap((val) => {
      console.log(`Valor original: ${val}`);

      // Agregar log a UI
      const logEntry = document.createElement('div');
      logEntry.textContent = `tap: El valor ${val} pasó`;
      logEntry.style.color = '#666';
      logOutput.appendChild(logEntry);
    }),
    map((val) => val * 10),
    tap((val) => {
      console.log(`Valor transformado: ${val}`);

      // Agregar log a UI
      const logEntry = document.createElement('div');
      logEntry.textContent = `tap: Valor transformado ${val}`;
      logEntry.style.color = '#090';
      logOutput.appendChild(logEntry);
    })
  )
  .subscribe((val) => {
    // Mostrar resultado final en UI
    const resultItem = document.createElement('div');
    resultItem.textContent = `Resultado: ${val}`;
    resultItem.style.fontWeight = 'bold';
    logOutput.appendChild(resultItem);
  });

```


## ✅ Resumen

- `tap` es un operador especializado para **insertar efectos secundarios**
- **La salida de log y las actualizaciones de UI** se pueden realizar sin cambiar el flujo de valores
- Se puede combinar con `finalize` y `catchError` para un control más práctico
