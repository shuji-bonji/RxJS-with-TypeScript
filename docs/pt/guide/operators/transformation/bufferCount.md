---
description: bufferCount é um operador de conversão RxJS que gera um array de valores para cada número especificado de itens. É ideal para processamento em lote, agregação de dados por contagem fixa, divisão de pacotes e outro controle de stream baseado em contagem, e permite operações de array type-safe através da inferência de tipo do TypeScript.
---

# bufferCount - Coleta Valores por Contagem Especificada

O operador `bufferCount` **agrupa** um número especificado de valores emitidos e os gera como um array.
Isso é útil para processamento em lote onde você deseja separar valores por contagem.

## 🔰 Sintaxe Básica e Uso

```ts
import { interval } from 'rxjs';
import { bufferCount } from 'rxjs';

// Emite valores a cada 100ms
const source$ = interval(100);

source$.pipe(
  bufferCount(5)
).subscribe(buffer => {
  console.log('Valores a cada 5:', buffer);
});

// Saída:
// Valores a cada 5: [0, 1, 2, 3, 4]
// Valores a cada 5: [5, 6, 7, 8, 9]
// ...
```

- Gera um array de 5 valores de cada vez.
- É único por agrupar em **base de contagem**, não em base de tempo.

[🌐 Documentação Oficial RxJS - `bufferCount`](https://rxjs.dev/api/operators/bufferCount)

## 💡 Padrões Típicos de Uso

- Dividir e enviar pacotes de dados
- Salvamento em lote ou processamento em lote por uma certa contagem
- Agregação de eventos de entrada por um certo número de ocorrências

## 🧠 Exemplo de Código Prático (com UI)

Este é um exemplo de exibir um resumo de pressionamentos de tecla do teclado a cada 5 pressionamentos.

```ts
import { fromEvent } from 'rxjs';
import { map, bufferCount } from 'rxjs';

// Criar área de saída
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Stream de evento de entrada de tecla
fromEvent<KeyboardEvent>(document, 'keydown').pipe(
  map(event => event.key),
  bufferCount(5)
).subscribe(keys => {
  const message = `5 entradas: ${keys.join(', ')}`;
  console.log(message);
  output.textContent = message;
});
```

- Cada vez que uma tecla é pressionada cinco vezes, esses cinco pressionamentos são exibidos juntos.
- Você pode experimentar o processo de agregação de acordo com a contagem.
