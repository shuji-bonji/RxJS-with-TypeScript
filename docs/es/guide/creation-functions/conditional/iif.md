---
description: "La Función de Creación iif es una función tipo operador ternario que selecciona entre dos Observables según una expresión condicional. Evalúa la condición al momento de suscripción y devuelve el Observable apropiado. Explicamos las diferencias con defer(), implementación segura de tipos en TypeScript y patrones de bifurcación dinámica."
---

# iif - Selección de Observable basada en condición

El operador `iif` selecciona entre dos Observables basándose en el resultado de la evaluación de una expresión condicional.
Tiene una funcionalidad similar al operador ternario de JavaScript (`condition ? trueValue : falseValue`).


## Sintaxis básica y comportamiento

```ts
import { iif, of } from 'rxjs';

function getData(condition: boolean) {
  return iif(() => condition, of('SÍ'), of('NO'));
}

getData(true).subscribe(console.log);

// Salida:
// SÍ
```

Si la condición es `true` se devuelve `'SÍ'`, si es `false` se devuelve `'NO'`.

[🌐 Documentación Oficial RxJS - iif](https://rxjs.dev/api/index/function/iif)

## Ejemplos de uso típicos

`iif` se usa frecuentemente combinado con `EMPTY` para devolver un "stream que no emite nada" cuando la condición no se cumple.

```ts
import { iif, of, EMPTY } from 'rxjs';

function conditionalData(value: number) {
  return iif(
    () => value > 0,
    of(`Valor positivo: ${value}`),
    EMPTY
  );
}

conditionalData(0).subscribe(console.log);
conditionalData(1).subscribe(console.log);

// Salida:
// Valor positivo: 1
```


## Ejemplo de código práctico (con UI)

En el siguiente ejemplo de código con UI, el contenido de emisión del Observable y si se emite o no se cambia dinámicamente usando `iif` según las operaciones del usuario o la entrada numérica.

Este tipo de patrón es adecuado para los siguientes casos de uso en producción:

- ✅ Suprimir solicitudes API según el valor de entrada (ej: no enviar si el número es 0 o menor)
- ✅ Cambiar la visualización de pantalla o el modo de procesamiento según flags de configuración
- ✅ Control de visualización de confirmación o modales basado en condiciones

```ts
import { iif, of, EMPTY } from 'rxjs';

function conditionalData(value: number) {
  return iif(() => value > 0, of(`Valor positivo: ${value}`), EMPTY);
}

// Devolver diferentes Observables basándose en la condición
function getDataBasedOnCondition(condition: boolean) {
  return iif(() => condition, of('La condición es true'), of('La condición es false'));
}

// Crear elementos UI
const iifContainer = document.createElement('div');
iifContainer.innerHTML = '<h3>Ejemplo del operador iif:</h3>';
document.body.appendChild(iifContainer);

const trueButton = document.createElement('button');
trueButton.textContent = 'Ejecutar con condición True';
trueButton.style.marginRight = '10px';
iifContainer.appendChild(trueButton);

const falseButton = document.createElement('button');
falseButton.textContent = 'Ejecutar con condición False';
iifContainer.appendChild(falseButton);

const iifResult = document.createElement('div');
iifResult.style.marginTop = '10px';
iifResult.style.padding = '10px';
iifResult.style.border = '1px solid #ddd';
iifContainer.appendChild(iifResult);

trueButton.addEventListener('click', () => {
  iifResult.textContent = '';
  getDataBasedOnCondition(true).subscribe((result) => {
    iifResult.textContent = result;
    iifResult.style.color = 'green';
  });
});

falseButton.addEventListener('click', () => {
  iifResult.textContent = '';
  getDataBasedOnCondition(false).subscribe((result) => {
    iifResult.textContent = result;
    iifResult.style.color = 'red';
  });
});

// Ejemplo de combinación con EMPTY (bifurcación condicional por número)
const emptyContainer = document.createElement('div');
emptyContainer.innerHTML = '<h3>Combinación de iif con EMPTY:</h3>';
document.body.appendChild(emptyContainer);

const valueInput = document.createElement('input');
valueInput.type = 'number';
valueInput.placeholder = 'Ingrese un número';
valueInput.style.marginRight = '10px';
emptyContainer.appendChild(valueInput);

const checkButton = document.createElement('button');
checkButton.textContent = 'Ejecutar';
emptyContainer.appendChild(checkButton);

const emptyResult = document.createElement('div');
emptyResult.style.marginTop = '10px';
emptyResult.style.padding = '10px';
emptyResult.style.border = '1px solid #ddd';
emptyContainer.appendChild(emptyResult);

checkButton.addEventListener('click', () => {
  const value = Number(valueInput.value);
  emptyResult.textContent = '';

  conditionalData(value).subscribe({
    next: (result) => {
      emptyResult.textContent = result;
      emptyResult.style.color = 'green';
    },
    complete: () => {
      if (!emptyResult.textContent) {
        emptyResult.textContent =
          'Se ingresó un valor de 0 o menor, por lo que no se emitió nada';
        emptyResult.style.color = 'gray';
      }
    },
  });
});

```
