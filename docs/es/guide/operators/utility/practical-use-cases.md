---
description: Esta presentación cubre casos de uso prácticos de operadores de utilidad de RxJS (tap, startWith, finalize, delay, timeout, retry, etc.). Se introducirán patrones prácticos usados frecuentemente en desarrollo de UI como gestión de estado de carga, validación de formularios reactivos, control de llamadas a API, manejo de errores, asistencia de depuración, etc. con ejemplos de código TypeScript. Aprenderá técnicas de implementación para controlar y observar el comportamiento del stream.
---

# Casos de Uso Prácticos

## Gestión de Estado de Carga

Este es un ejemplo de usar `tap`, `finalize`, etc. para gestionar el estado de carga.

```ts
import { of, throwError } from 'rxjs';
import { tap, delay, finalize, catchError } from 'rxjs';

// Elementos de UI
const loadingExample = document.createElement('div');
loadingExample.innerHTML = '<h3>Llamada a API y gestión de estado de carga:</h3>';
document.body.appendChild(loadingExample);

// Indicador de carga
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = 'Cargando...';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.backgroundColor = '#e3f2fd';
loadingIndicator.style.borderRadius = '5px';
loadingIndicator.style.display = 'none';
loadingExample.appendChild(loadingIndicator);

// Área de visualización de datos
const dataContainer = document.createElement('div');
dataContainer.style.marginTop = '10px';
dataContainer.style.padding = '10px';
dataContainer.style.border = '1px solid #ddd';
dataContainer.style.borderRadius = '5px';
dataContainer.style.minHeight = '100px';
loadingExample.appendChild(dataContainer);

// Botón de éxito
const successButton = document.createElement('button');
successButton.textContent = 'Solicitud exitosa';
successButton.style.marginRight = '10px';
successButton.style.padding = '8px 16px';
loadingExample.insertBefore(successButton, loadingIndicator);

// Botón de fallo
const failButton = document.createElement('button');
failButton.textContent = 'Solicitud fallida';
failButton.style.padding = '8px 16px';
loadingExample.insertBefore(failButton, loadingIndicator);

// Simular solicitud exitosa de API
function simulateSuccessRequest() {
  return of({
    id: 1,
    name: 'Datos de muestra',
    description: 'Estos son datos recuperados de la API.'
  }).pipe(
    // Mostrar carga al inicio de solicitud
    tap(() => {
      loadingIndicator.style.display = 'block';
      dataContainer.innerHTML = '';
    }),
    // Simular latencia de API
    delay(1500),
    // Siempre ocultar carga al completarse la solicitud
    finalize(() => {
      loadingIndicator.style.display = 'none';
    })
  );
}

// Simular solicitud fallida de API
function simulateFailRequest() {
  return throwError(() => new Error('Solicitud de API fallida')).pipe(
    // Mostrar carga al inicio de solicitud
    tap(() => {
      loadingIndicator.style.display = 'block';
      dataContainer.innerHTML = '';
    }),
    // Simular latencia de API
    delay(1500),
    // Manejo de errores
    catchError(error => {
      const errorElement = document.createElement('div');
      errorElement.textContent = `Error: ${error.message}`;
      errorElement.style.color = 'red';
      dataContainer.appendChild(errorElement);

      return throwError(() => error);
    }),
    // Siempre ocultar carga al completarse la solicitud
    finalize(() => {
      loadingIndicator.style.display = 'none';
    })
  );
}

// Clic en botón de éxito
successButton.addEventListener('click', () => {
  // Deshabilitar botones
  successButton.disabled = true;
  failButton.disabled = true;

  simulateSuccessRequest().subscribe({
    next: data => {
      // Mostrar datos
      const dataElement = document.createElement('div');
      dataElement.innerHTML = `
        <h4>${data.name}</h4>
        <p>${data.description}</p>
        <p><em>ID: ${data.id}</em></p>
      `;
      dataContainer.appendChild(dataElement);
    },
    error: err => {
      console.error('Error:', err);
    },
    complete: () => {
      // Reactivar botones
      successButton.disabled = false;
      failButton.disabled = false;
    }
  });
});

// Clic en botón de fallo
failButton.addEventListener('click', () => {
  // Deshabilitar botones
  successButton.disabled = true;
  failButton.disabled = true;

  simulateFailRequest().subscribe({
    next: () => {
      // No tendrá éxito, pero por si acaso
    },
    error: () => {
      // Error ya manejado por catchError
      console.log('Manejo de error completado');
    },
    complete: () => {
      // Reactivar botones
      successButton.disabled = false;
      failButton.disabled = false;
    }
  });
});
```

## Validación y Envío de Formularios

El siguiente es un ejemplo de implementar validación y envío de formularios usando `startWith`, `tap`, `finalize`, etc.

```ts
import { fromEvent, combineLatest, of } from 'rxjs';
import { map, startWith, debounceTime, tap, finalize, catchError, delay } from 'rxjs';

// UI de formulario
const formExample = document.createElement('div');
formExample.innerHTML = '<h3>Ejemplo de formulario reactivo:</h3>';
document.body.appendChild(formExample);

// Crear elementos de formulario
const form = document.createElement('form');
form.style.padding = '15px';
form.style.border = '1px solid #ddd';
form.style.borderRadius = '5px';
formExample.appendChild(form);

// Campo de entrada de nombre
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Nombre: ';
nameLabel.style.display = 'block';
nameLabel.style.marginBottom = '5px';
form.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.style.padding = '8px';
nameInput.style.width = '100%';
nameInput.style.marginBottom = '15px';
form.appendChild(nameInput);

const nameError = document.createElement('div');
nameError.style.color = 'red';
nameError.style.fontSize = '12px';
nameError.style.marginTop = '-10px';
nameError.style.marginBottom = '15px';
form.appendChild(nameError);

// Campo de entrada de correo electrónico
const emailLabel = document.createElement('label');
emailLabel.textContent = 'Correo electrónico: ';
emailLabel.style.display = 'block';
emailLabel.style.marginBottom = '5px';
form.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.style.padding = '8px';
emailInput.style.width = '100%';
emailInput.style.marginBottom = '15px';
form.appendChild(emailInput);

const emailError = document.createElement('div');
emailError.style.color = 'red';
emailError.style.fontSize = '12px';
emailError.style.marginTop = '-10px';
emailError.style.marginBottom = '15px';
form.appendChild(emailError);

// Botón de envío
const submitButton = document.createElement('button');
submitButton.type = 'submit';
submitButton.textContent = 'Enviar';
submitButton.style.padding = '8px 16px';
submitButton.disabled = true; // Inicialmente deshabilitado
form.appendChild(submitButton);

// Área de visualización de resultados
const formResult = document.createElement('div');
formResult.style.marginTop = '20px';
formResult.style.padding = '10px';
formResult.style.border = '1px solid transparent';
formResult.style.borderRadius = '5px';
formResult.style.display = 'none';
formExample.appendChild(formResult);

// Validación de entrada de nombre
const name$ = fromEvent(nameInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  startWith(''),
  debounceTime(300),
  map(value => {
    if (!value) {
      return { value, valid: false, error: 'El nombre es obligatorio' };
    }
    if (value.length < 2) {
      return { value, valid: false, error: 'El nombre debe tener al menos 2 caracteres' };
    }
    return { value, valid: true, error: null };
  })
);

// Validación de entrada de correo electrónico
const emailRegex = /^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$/;
const email$ = fromEvent(emailInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  startWith(''),
  debounceTime(300),
  map(value => {
    if (!value) {
      return { value, valid: false, error: 'El correo electrónico es obligatorio' };
    }
    if (!emailRegex.test(value)) {
      return { value, valid: false, error: 'Por favor ingrese un correo electrónico válido' };
    }
    return { value, valid: true, error: null };
  })
);

// Monitorear estado de validación de todo el formulario
combineLatest([name$, email$]).pipe(
  map(([nameState, emailState]) => {
    // ¿Es válido todo el formulario?
    const isValid = nameState.valid && emailState.valid;

    // Mostrar errores de validación
    nameError.textContent = nameState.error || '';
    emailError.textContent = emailState.error || '';

    return isValid;
  })
).subscribe(isValid => {
  // Habilitar/deshabilitar botón de envío
  submitButton.disabled = !isValid;
});

// Procesamiento de envío de formulario
fromEvent(form, 'submit').pipe(
  tap(event => {
    // Prevenir envío predeterminado del formulario
    event.preventDefault();

    // Establecer en estado de envío
    submitButton.disabled = true;
    submitButton.textContent = 'Enviando...';

    // Restablecer área de visualización de resultados
    formResult.style.display = 'none';
  }),
  // Obtener datos del formulario
  map(() => ({
    name: nameInput.value.trim(),
    email: emailInput.value.trim()
  })),
  // Simular solicitud de API
  delay(1500),
  // Siempre volver al estado de envío completado
  finalize(() => {
    submitButton.textContent = 'Enviar';
    submitButton.disabled = false;
  }),
  // Manejo de errores
  catchError(error => {
    formResult.textContent = `Error: ${error.message}`;
    formResult.style.display = 'block';
    formResult.style.backgroundColor = '#ffebee';
    formResult.style.borderColor = '#f44336';

    return of(null); // Continuar stream
  })
).subscribe(data => {
  if (data) {
    // Envío exitoso
    formResult.innerHTML = `
      <div style="font-weight: bold;">¡Envío exitoso!</div>
      <div>Nombre: ${data.name}</div>
      <div>Correo electrónico: ${data.email}</div>
    `;
    formResult.style.display = 'block';
    formResult.style.backgroundColor = '#e8f5e9';
    formResult.style.borderColor = '#4caf50';

    // Restablecer formulario
    nameInput.value = '';
    emailInput.value = '';
  }
});
```

## Cómo Elegir un Operador de Utilidad

| Propósito | Operador | Situación de Uso |
|------|--------------|---------|
| Ejecución de efecto secundario | `tap` | Depuración, salida de log, actualización de UI, etc. |
| Retardo de salida de valores | `delay` | Animación, ajuste de temporización, etc. |
| Configuración de tiempo de espera | `timeout` | Tiempo de espera para solicitudes de API, procesamiento asíncrono |
| Procesamiento al completarse | `finalize` | Limpieza de recursos, liberar estado de carga |
| Establecer valor inicial | `startWith` | Inicializar estado, mostrar marcadores de posición |
| Convertir a array | `toArray` | Procesamiento por lotes, todos los resultados se procesan juntos |
| Reintentar en caso de error | `retry` | Solicitudes de red, recuperarse de errores temporales |
| Repetir stream | `repeat` | Polling, procesamiento periódico |

## Resumen

Los operadores de utilidad son herramientas importantes que hacen que la programación en RxJS sea más eficiente y robusta. La combinación adecuada de estos operadores proporciona los siguientes beneficios:

1. **Facilidad de Depuración**: Usando `tap`, puede verificar fácilmente el estado intermedio del stream.
2. **Tolerancia a Errores**: La combinación de `retry`, `timeout` y `catchError` proporciona un manejo de errores robusto.
3. **Gestión de Recursos**: `finalize` se puede usar para garantizar una limpieza adecuada de recursos.
4. **Mejora de capacidad de respuesta de UI**: `startWith`, `delay`, etc. se pueden usar para mejorar la experiencia del usuario.
5. **Mejorar legibilidad del código**: El uso de operadores de utilidad puede separar claramente los efectos secundarios de la conversión pura de datos.

Estos operadores demuestran su verdadero valor cuando se usan en combinación con otros operadores en lugar de solos. En el desarrollo real de aplicaciones, es común combinar múltiples operadores para gestionar flujos de procesamiento asíncronos complejos.
