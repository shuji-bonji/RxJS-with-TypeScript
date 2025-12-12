---
description: toArray é um operador utilitário do RxJS que combina todos os valores emitidos até que o Observable seja completado em um único array. É ideal para situações onde você deseja tratar todo o stream como um array, como processamento em lote, exibição de UI após aquisição em lote e processamento agregado. Como acumula valores até a conclusão, não pode ser usado com streams infinitos.
---

# toArray - Converter Valores em Array

O operador `toArray` é um operador que **combina todos os valores emitidos pelo Observable até a conclusão em um único array**.
Isso é útil para processamento em lote, exibição de UI após recuperação em lote, agregação, etc.


## 🔰 Sintaxe Básica e Operação

```ts
import { of } from 'rxjs';
import { toArray } from 'rxjs';

of(1, 2, 3).pipe(
  toArray()
).subscribe(console.log);

// Saída:
// [1, 2, 3]
```

Todos os valores são combinados em um único array, que é emitido quando o Observable é completado.

[🌐 Documentação Oficial do RxJS - toArray](https://rxjs.dev/api/index/function/toArray)

## 💡 Exemplo de Uso Típico

Isso pode ser usado em situações onde você deseja processar vários resultados assíncronos de uma vez ou exibi-los na UI em lote.

```ts
import { interval, of } from 'rxjs';
import { take, toArray, delayWhen, delay } from 'rxjs';

interval(500)
  .pipe(
    take(5),
    delayWhen((val) => of(val).pipe(delay(val * 200))),
    toArray()
  )
  .subscribe((result) => {
    console.log('Receber todos na conclusão:', result);
  });

// Saída:
// Receber todos na conclusão: [0, 1, 2, 3, 4]
```


## 🧪 Exemplo de Código Prático (com UI)

```ts
import { interval } from 'rxjs';
import { take, toArray } from 'rxjs';

// Área de exibição de saída
const toArrayOutput = document.createElement('div');
toArrayOutput.innerHTML = '<h3>Exemplo de toArray:</h3>';
document.body.appendChild(toArrayOutput);

// Área de exibição de valores individuais
const individualValues = document.createElement('div');
individualValues.innerHTML = '<h4>Valores Individuais:</h4>';
toArrayOutput.appendChild(individualValues);

// Área de exibição de resultado do array
const arrayResult = document.createElement('div');
arrayResult.innerHTML = '<h4>Resultado do Array:</h4>';
arrayResult.style.marginTop = '20px';
toArrayOutput.appendChild(arrayResult);

// Inscrever-se em valores individuais
interval(500)
  .pipe(take(5))
  .subscribe((val) => {
    const valueItem = document.createElement('div');
    valueItem.textContent = `Valor: ${val}`;
    individualValues.appendChild(valueItem);
  });

// Inscrever-se no mesmo stream como array
interval(500)
  .pipe(take(5), toArray())
  .subscribe((array) => {
    const resultItem = document.createElement('div');
    resultItem.textContent = `Array de resultado: [${array.join(', ')}]`;
    resultItem.style.fontWeight = 'bold';
    resultItem.style.padding = '10px';
    resultItem.style.backgroundColor = '#f5f5f5';
    resultItem.style.borderRadius = '5px';
    arrayResult.appendChild(resultItem);

    // Exibir elementos do array individualmente
    const arrayItems = document.createElement('div');
    arrayItems.style.marginTop = '10px';

    array.forEach((item, index) => {
      const arrayItem = document.createElement('div');
      arrayItem.textContent = `array[${index}] = ${item}`;
      arrayItems.appendChild(arrayItem);
    });

    arrayResult.appendChild(arrayItems);
  });
```


## ✅ Resumo

- `toArray` **emite um array de todos os valores na conclusão**
- Ideal para situações onde você deseja lidar com todo o stream de forma agregada
- Combinado com `concatMap`, `delay`, etc., pode ser usado para **processamento sequencial em lote assíncrono**
