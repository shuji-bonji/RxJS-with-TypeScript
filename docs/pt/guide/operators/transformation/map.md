---
description: O operador map é um método básico de conversão que aplica uma função a cada valor em um Observable para gerar um novo valor, e é frequentemente usado para formatação de formulários e processamento de respostas de API.
---

# map - Aplica uma Função de Conversão a Cada Valor

O operador `map` aplica uma função especificada a **cada valor** no stream para produzir um novo valor após a conversão.
Semelhante ao método `Array.prototype.map` de um array, mas funciona **em streams assíncronos**.


## 🔰 Sintaxe Básica e Uso

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3).pipe(
  map(value => value * 10)
).subscribe(console.log);
// Saída: 10, 20, 30
```

Aplica a função `value => value * 10` a cada valor para produzir um novo valor.

[🌐 Documentação Oficial RxJS - map](https://rxjs.dev/api/index/function/map)


## 💡 Padrões Típicos de Uso
- Conversão de resposta de API (extrair apenas propriedades necessárias)
- Formatação de dados de entrada de formulário
- Processamento de números e strings em streams
- Extrair apenas dados necessários de eventos de UI


## 🧠 Exemplo de Código Prático (com UI)

Este é um exemplo de exibição de um valor numérico de entrada multiplicado por 2 em tempo real.

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Criar campo de entrada
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Digite um número';
document.body.appendChild(input);

// Criar campo de saída
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Stream de evento de entrada
fromEvent(input, 'input').pipe(
  map(event => Number((event.target as HTMLInputElement).value)),
  map(value => value * 2)
).subscribe(result => {
  output.textContent = `Valor dobrado: ${result}`;
});
```

- O valor de entrada é dobrado em tempo real e exibido.
- Uma cadeia simples de conversão de dados é realizada aplicando map continuamente.
