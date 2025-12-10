---
description: finalize es un operador de utilidad de RxJS que define un proceso que se ejecuta cada vez que se completa, se produce un error o se cancela la suscripción de un Observable. Es ideal para situaciones que requieren limpieza al final del stream, como liberación de recursos, finalización de visualización de carga y operaciones de limpieza. Garantiza que las operaciones se ejecuten de manera tan confiable como try-finally y ayuda a prevenir fugas de memoria.
---

# finalize - Procesamiento al Completarse

El operador `finalize` define un proceso que se llama cada vez que **Observable se completa, presenta errores o se cancela la suscripción**.
Esto es ideal para procesos que "deben ejecutarse" como limpieza y liberación de carga de UI.

## 🔰 Sintaxis Básica y Operación

```ts
import { of } from 'rxjs';
import { finalize } from 'rxjs';

of('Completado')
  .pipe(finalize(() => console.log('El stream ha terminado')))
  .subscribe(console.log);
// Salida:
// Completado
// El stream ha terminado
```

En este ejemplo, el proceso en `finalize` se ejecuta después de emitir un valor en `of()`.
**Se llama de manera confiable tanto para `complete` como para `error`**.

[🌐 Documentación Oficial de RxJS - finalize](https://rxjs.dev/api/index/function/finalize)

## 💡 Ejemplo de Uso Típico

El siguiente es un ejemplo de cambiar la visualización de carga antes y después del streaming.

```ts
import { of } from 'rxjs';
import { tap, delay, finalize } from 'rxjs';

let isLoading = false;

of('Datos')
  .pipe(
    tap(() => {
      isLoading = true;
      console.log('Carga iniciada');
    }),
    delay(1000),
    finalize(() => {
      isLoading = false;
      console.log('Carga finalizada');
    })
  )
  .subscribe((value) => console.log('Recuperado:', value));
// Salida:
// Carga iniciada
// Recuperado: Datos
// Carga finalizada
```

## 🧪 Ejemplo de Código Práctico (con UI)

```ts
import { interval } from 'rxjs';
import { take, finalize, tap } from 'rxjs';

// Área de visualización de salida
const finalizeOutput = document.createElement('div');
finalizeOutput.innerHTML = '<h3>Ejemplo de finalize:</h3>';
document.body.appendChild(finalizeOutput);

// Indicador de carga
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = 'Cargando datos...';
loadingIndicator.style.backgroundColor = '#e0f7fa';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.borderRadius = '5px';
finalizeOutput.appendChild(loadingIndicator);

// Visualización de progreso
const progressContainer = document.createElement('div');
progressContainer.style.marginTop = '10px';
finalizeOutput.appendChild(progressContainer);

// Elemento de mensaje de finalización
const completionMessage = document.createElement('div');
completionMessage.style.marginTop = '10px';
completionMessage.style.fontWeight = 'bold';
finalizeOutput.appendChild(completionMessage);

// Simulación de recuperación de datos
interval(500)
  .pipe(
    take(5), // Recuperar 5 valores
    tap((val) => {
      const progressItem = document.createElement('div');
      progressItem.textContent = `Procesando elemento ${val + 1}...`;
      progressContainer.appendChild(progressItem);
    }),
    finalize(() => {
      loadingIndicator.style.display = 'none';
      completionMessage.textContent = '¡Procesamiento completado!';
      completionMessage.style.color = 'green';
    })
  )
  .subscribe({
    complete: () => {
      const successMsg = document.createElement('div');
      successMsg.textContent = 'Todos los datos cargados exitosamente.';
      completionMessage.appendChild(successMsg);
    },
  });
```

## ✅ Resumen

- `finalize` **siempre se ejecuta** independientemente de completarse, error o terminación manual
- Ideal para limpieza y procesos de terminación de carga
- Se puede combinar con otros operadores (`tap`, `delay`, etc.) para **limpieza asíncrona segura**
