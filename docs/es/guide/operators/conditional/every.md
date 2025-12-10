---
description: El operador every evalúa si todos los valores cumplen una condición especificada y puede realizar evaluación de cortocircuito devolviendo false tan pronto como se encuentra el primer valor que no cumple la condición.
---

# every - Verificar si Todos los Valores Cumplen la Condición

El operador `every` evalúa si todos los valores emitidos desde el Observable fuente cumplen una condición especificada, y **devuelve `false` y termina tan pronto como se encuentra el primer valor que no cumple la condición**. Si todos los valores cumplen la condición, se devuelve `true`.

## 🔰 Sintaxis Básica y Operación

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

from([2, 4, 6, 8])
  .pipe(
    every((x) => x % 2 === 0)
  )
  .subscribe(console.log);
// Salida: true
```

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

from([2, 4, 5, 8])
  .pipe(
    every((x) => x % 2 === 0)
  )
  .subscribe(console.log);
// Salida: false (se detiene en 5)
```

[🌐 RxJS Official Documentation - every](https://rxjs.dev/api/index/function/every)

## 💡 Ejemplos de Uso Típicos

- **Verificación de validación**: Confirmar que se cumplen todas las condiciones
- **Validación de entrada masiva**: Evaluar múltiples valores juntos
- **Diferente del filtro de array, útil para verificar la satisfacción general de una vez**

## 🧪 Ejemplos de Código Prácticos (con UI)

### ✅ 1. Determinar si el Array Tiene Solo Números Pares

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

const container = document.createElement('div');
container.innerHTML = '<h3>Ejemplo del operador every:</h3>';
document.body.appendChild(container);

const allEvenButton = document.createElement('button');
allEvenButton.textContent = 'Solo pares [2, 4, 6, 8]';
container.appendChild(allEvenButton);

const someOddButton = document.createElement('button');
someOddButton.textContent = 'Contiene impares [2, 4, 5, 8]';
container.appendChild(someOddButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
container.appendChild(result);

allEvenButton.addEventListener('click', () => {
  result.textContent = 'Verificando...';
  from([2, 4, 6, 8])
    .pipe(every((x) => x % 2 === 0))
    .subscribe((res) => {
      result.textContent = `¿Todos pares?: ${res}`;
      result.style.color = res ? 'green' : 'red';
    });
});

someOddButton.addEventListener('click', () => {
  result.textContent = 'Verificando...';
  from([2, 4, 5, 8])
    .pipe(every((x) => x % 2 === 0))
    .subscribe((res) => {
      result.textContent = `¿Todos pares?: ${res}`;
      result.style.color = res ? 'green' : 'red';
    });
});
```

### ✅ 2. Utilización en Validación de Formularios

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, startWith, every, tap } from 'rxjs';

// Crear elementos UI
const formContainer = document.createElement('div');
formContainer.innerHTML = '<h3>Validación de formulario con every:</h3>';
document.body.appendChild(formContainer);

// Crear formulario
const form = document.createElement('form');
form.style.padding = '15px';
form.style.border = '1px solid #ddd';
form.style.borderRadius = '5px';
formContainer.appendChild(form);

// Entrada de nombre
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Nombre: ';
nameLabel.style.display = 'block';
nameLabel.style.marginBottom = '5px';
form.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.style.width = '100%';
nameInput.style.padding = '5px';
nameInput.style.marginBottom = '15px';
form.appendChild(nameInput);

// Entrada de edad
const ageLabel = document.createElement('label');
ageLabel.textContent = 'Edad: ';
ageLabel.style.display = 'block';
ageLabel.style.marginBottom = '5px';
form.appendChild(ageLabel);

const ageInput = document.createElement('input');
ageInput.type = 'number';
ageInput.min = '0';
ageInput.style.width = '100%';
ageInput.style.padding = '5px';
ageInput.style.marginBottom = '15px';
form.appendChild(ageInput);

// Entrada de email
const emailLabel = document.createElement('label');
emailLabel.textContent = 'Email: ';
emailLabel.style.display = 'block';
emailLabel.style.marginBottom = '5px';
form.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.style.width = '100%';
emailInput.style.padding = '5px';
emailInput.style.marginBottom = '15px';
form.appendChild(emailInput);

// Botón de envío
const submitButton = document.createElement('button');
submitButton.type = 'submit';
submitButton.textContent = 'Enviar';
submitButton.disabled = true;
form.appendChild(submitButton);

// Mensaje de validación
const validationMessage = document.createElement('div');
validationMessage.style.marginTop = '10px';
validationMessage.style.color = 'red';
formContainer.appendChild(validationMessage);

// Validación de nombre
const nameValid$ = fromEvent(nameInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return value.length >= 2;
  }),
  startWith(false)
);

// Validación de edad
const ageValid$ = fromEvent(ageInput, 'input').pipe(
  map((event) => {
    const value = Number((event.target as HTMLInputElement).value);
    return !isNaN(value) && value > 0 && value < 120;
  }),
  startWith(false)
);

// Validación de email
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
const emailValid$ = fromEvent(emailInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return emailRegex.test(value);
  }),
  startWith(false)
);

// Validar todos los campos
combineLatest([nameValid$, ageValid$, emailValid$])
  .pipe(
    // tap((v) => console.log(v)),
    map((validList) => validList.every((v) => v === true))
  )
  .subscribe((allValid) => {
    submitButton.disabled = !allValid;
    if (allValid) {
      validationMessage.textContent = '';
    } else {
      validationMessage.textContent =
        'Por favor completa todos los campos correctamente';
    }
  });

// Envío del formulario
form.addEventListener('submit', (event) => {
  event.preventDefault();

  const formData = {
    name: nameInput.value,
    age: ageInput.value,
    email: emailInput.value,
  };

  validationMessage.textContent = '¡Formulario enviado!';
  validationMessage.style.color = 'green';

  console.log('Datos enviados:', formData);
});
```
